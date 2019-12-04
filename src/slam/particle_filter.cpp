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
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
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
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    
    std::vector<particle_t> prior;
    std::vector<double> weight_accumlate;
    for(unsigned int i = 0; i < posterior_.size(); i++) {
        weight_accumlate[i] = posterior_[i].weight;
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

    return prior;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal;
    for(unsigned int i = 0; i < proposal.size(); i++) {
        proposal.emplace_back(actionModel_.applyAction(prior[i]));
    }
    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the 
    ///////////       particles in the proposal distribution
    std::vector<particle_t> posterior;
    double weight_sum = 0;
    std::vector<double> weights;
    for(unsigned int i = 0; i < proposal.size(); i++) {
        double likelihood = sensorModel_.likelihood(proposal[i], laser, map);
        weight_sum += likelihood;
        weights.emplace_back(likelihood);
    }

    // set the normalized likelihood
    for(unsigned int i = 0; i < proposal.size(); i++) {
        particle_t posterior_particle;
        posterior_particle.pose = proposal[i].pose;
        posterior_particle.parent_pose = proposal[i].pose;
        posterior_particle.weight = weights[i] / weight_sum;
        posterior.emplace_back(posterior_particle);
    }

    return posterior;
}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    pose_xyt_t pose;
    for(unsigned int i = 0; i < posterior.size(); i++) {
        pose.x += posterior[i].pose.x * posterior[i].weight;
        pose.y += posterior[i].pose.y * posterior[i].weight;
        pose.theta += posterior[i].pose.theta * posterior[i].weight;
    }
    
    // TODO: do I need to change theta into [0, 2pi] ?
    return pose;
}
