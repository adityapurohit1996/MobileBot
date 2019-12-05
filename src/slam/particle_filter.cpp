#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>


ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> x_d{pose.x, pose_sigmas_[0]};
    std::normal_distribution<> y_d{pose.y, pose_sigmas_[1]};
    std::normal_distribution<> theta_d{pose.theta, pose_sigmas_[2]};
    posterior_.reserve(kNumParticles_);
    for(int i = 0; i < kNumParticles_; i++) {
        particle_t sample_particle;
        sample_particle.pose.x = x_d(gen);
        sample_particle.pose.y = y_d(gen);
        sample_particle.pose.theta = theta_d(gen);
        sample_particle.weight = 1.0 / kNumParticles_;
        posterior_.emplace_back(sample_particle);
    }
    std::cout << "Initialization Finished" << std::endl;
}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    std::cout << hasRobotMoved << std::endl;
    if(hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        posteriorPose_ = estimatePosteriorPose(posterior_);
    }
    
    posteriorPose_.utime = odometry.utime;
    
    return posteriorPose_;
}


pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


particles_t ParticleFilter::particles(void) const
{
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    std::cout << "Resampling Started" << std::endl;
    std::vector<particle_t> prior;
    std::vector<double> weight_accumlate;
    for(unsigned int i = 0; i < posterior_.size(); i++) {
        weight_accumlate.emplace_back(posterior_[i].weight);
        if(i >= 1) {
            weight_accumlate[i] += weight_accumlate[i - 1];
        }
    }

    // re-sampling
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<double> distribution(0.0, 1.0);

    for(int i = 0; i < kNumParticles_; i++) {
        double random_val = distribution(gen);
        int resample_index = 0;
        for(int j = 0; j < kNumParticles_; j++) {
            if(random_val < weight_accumlate[j]) {
                break;
            }
            resample_index ++;
        }
        particle_t resample_particle;
        resample_particle.pose = posterior_[resample_index].pose;
        resample_particle.parent_pose = posterior_[resample_index].parent_pose;
        resample_particle.weight = 1.0 / (1.0 * kNumParticles_);
        prior.emplace_back(resample_particle);
    }
    std::cout << "Resampling Finished" << std::endl;
    return prior;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    std::vector<particle_t> proposal;
    for(int i = 0; i < kNumParticles_; i++) {
        proposal.emplace_back(actionModel_.applyAction(prior[i]));
    }
    std::cout << "Proposal Finished" << std::endl;
    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    std::vector<particle_t> posterior;
    double weight_sum = 0;
    std::vector<double> weights;
    for(int i = 0; i < kNumParticles_; i++) {
        double likelihood = sensorModel_.likelihood(proposal[i], laser, map);
        std::cout << likelihood << std::endl;
        weight_sum += likelihood;
        weights.emplace_back(likelihood);
    }

    // set the normalized likelihood
    for(int i = 0; i < kNumParticles_; i++) {
        particle_t posterior_particle;
        posterior_particle.pose = proposal[i].pose;
        posterior_particle.parent_pose = proposal[i].pose;
        posterior_particle.weight = weights[i] / weight_sum;
        posterior.emplace_back(posterior_particle);
    }
    std::cout << "Posterior Finished" << std::endl;
    return posterior;
}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    pose_xyt_t pose;
    pose.x = 0.0;
    pose.y = 0.0;
    pose.theta = 0.0;
    for(int i = 0; i < kNumParticles_; i++) {
        pose.x += posterior[i].pose.x * posterior[i].weight;
        pose.y += posterior[i].pose.y * posterior[i].weight;
        pose.theta += posterior[i].pose.theta * posterior[i].weight;
    }
    std::cout << "Estimation Finished" << std::endl;
    // TODO: do I need to change theta into [0, 2pi] ?
    return pose;
}
