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
    std::normal_distribution<double> x_d{pose.x, pose_sigmas_[0]};
    std::normal_distribution<double> y_d{pose.y, pose_sigmas_[1]};
    std::normal_distribution<double> theta_d{pose.theta, pose_sigmas_[2]};
    for(int i = 0; i < kNumParticles_; i++) {
        particle_t sample_particle;
        sample_particle.pose.x = x_d(gen);
        sample_particle.pose.y = y_d(gen);
        sample_particle.pose.theta = theta_d(gen);
        sample_particle.weight = 1.0 / kNumParticles_;
        sample_particle.pose.utime = pose.utime;
        posterior_[i] = sample_particle;
    }
    // posterior pose
    posteriorPose_.x = pose.x;
    posteriorPose_.y = pose.y;
    posteriorPose_.theta = pose.theta;
    posteriorPose_.utime = pose.utime;
}

void ParticleFilter::KidnappedInitialization(const OccupancyGrid& map, int64_t utime) {
    std::random_device rd{};
    std::mt19937 gen{rd()};

    float* map_limitation = map.MapLimitation();
    std::uniform_real_distribution<double> x_d(map_limitation[0], map_limitation[1]);
    std::uniform_real_distribution<double> y_d(map_limitation[2], map_limitation[3]);
    std::uniform_real_distribution<double> theta_d(-M_PI, M_PI);

    Point<float> map_origin = map.originInGlobalFrame();

    for(int i = 0; i < kNumParticles_; i++) {
        particle_t sample_particle;
        sample_particle.pose.x = x_d(gen);
        sample_particle.pose.y = y_d(gen);
        sample_particle.pose.theta = theta_d(gen);
        sample_particle.weight = 1.0 / kNumParticles_;
        sample_particle.pose.utime = utime;
        posterior_[i] = sample_particle;
    }
    // posterior pose
    posteriorPose_.x = map_origin.x;
    posteriorPose_.y = map_origin.y;
    posteriorPose_.theta = 0.0;
    posteriorPose_.utime = utime;
};

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
        // sensorModel_.ShowHit(sensorModel_.previous_ground_pose_, sensorModel_.ground_true_pose_, laser, map);
        // show Laser
        // sensorModel_.ShowLaser(sensorModel_.previous_ground_pose_, sensorModel_.ground_true_pose_, laser);
        // sensorModel_.ShowLaser(sensorModel_.map_particle_.parent_pose, sensorModel_.map_particle_.pose, laser);
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
    
    // particles for debug
    // particles.num_particles = sensorModel_.debug_particles_.size();
    // particles.particles = sensorModel_.debug_particles_;
    return particles;
}

double ParticleFilter::UpdateGroundTruth(const pose_xyt_t& pose, const lidar_t& scan){
    sensorModel_.UpdateGroundTruth(pose, scan);
};

std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    std::vector<particle_t> prior;
    std::vector<double> weight_accumlate;
    for(unsigned int i = 0; i < posterior_.size(); i++) {
        weight_accumlate.emplace_back(posterior_[i].weight);
        if(i >= 1) {
            weight_accumlate[i] += weight_accumlate[i - 1];
        }
    }

    // re-sampling ?
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
        //resample_index = i;
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
    std::vector<particle_t> proposal;
    for(int i = 0; i < kNumParticles_; i++) {
        proposal.emplace_back(actionModel_.applyAction(prior[i]));
    }
    return proposal;
}

std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    std::vector<particle_t> posterior;
    double weight_sum = 0;
    std::vector<double> weights;
    // Test Gt likelihood
    // double gt_likelihood = sensorModel_.GroundTruthLikelihood(laser, map);
    // std::cout << "Gt L :" << gt_likelihood << std::endl;
    for(int i = 0; i < kNumParticles_; i++) {
        double likelihood = sensorModel_.likelihood(proposal[i], laser, map);
        weight_sum += likelihood;
        weights.emplace_back(likelihood);
    }

    // Update map_particle
    sensorModel_.map_particle_.weight = 0.0;
    // set the normalized likelihood
    for(int i = 0; i < kNumParticles_; i++) {
        particle_t posterior_particle;
        posterior_particle.pose = proposal[i].pose;
        posterior_particle.parent_pose = proposal[i].parent_pose;
        posterior_particle.weight = weights[i] / weight_sum;
        posterior.emplace_back(posterior_particle);
        // Compare map_paricle
        if(posterior_particle.weight > sensorModel_.map_particle_.weight) {
            sensorModel_.map_particle_.pose = posterior_particle.pose;
            sensorModel_.map_particle_.parent_pose = posterior_particle.parent_pose;
            sensorModel_.map_particle_.weight = posterior_particle.weight;
        }
    }
    // update averge map likelihood
    double map_likelihood = sensorModel_.map_particle_.weight * weight_sum;
    std::cout << "MAP L:" << map_likelihood << std::endl;

    if((!sensorModel_.JudgeMeasurement(map_likelihood)) || map_likelihood < 1e-5) {
        // If measurements bad, do not update weights
        for(auto particle : posterior) {
            particle.weight = 1.0 / kNumParticles_;
        }
    }
    return posterior;
}

pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    pose_xyt_t pose;
    pose.x = 0.0;
    pose.y = 0.0;
    pose.theta = 0.0;

    double weight_sum = 1e-8;
    double average_angle = 0.0;
    double sample_angle = 0.0;

    // All pose has been transformed into (-pi, pi)
    std::cout << "Compute Theta" << std::endl;
    for(int i = 0; i < kNumParticles_; i++) {
        pose.x += posterior[i].pose.x * posterior[i].weight;
        pose.y += posterior[i].pose.y * posterior[i].weight;
        // transform angle
        average_angle = pose.theta / weight_sum;
        sample_angle = posterior[i].pose.theta;
        if((average_angle - sample_angle) >= M_PI) {
            sample_angle += 2 * M_PI;
        }
        if((average_angle - sample_angle) <= -M_PI) {
            sample_angle -= 2 * M_PI;
        }

        pose.theta += sample_angle * posterior[i].weight;
        weight_sum += posterior_[i].weight;
        // std::cout << "theta :" << sample_angle << ", W:" << posterior_[i].weight << std::endl;
    }

    // // Another way of averaging theta
    // double average_sin = 0.0;
    // double average_cos = 0.0;
    // for(int i = 0; i < kNumParticles_; i++) {
    //     pose.x += posterior[i].pose.x * posterior[i].weight;
    //     pose.y += posterior[i].pose.y * posterior[i].weight;
    //     // transform angle
    //     average_sin = sin(posterior[i].pose.theta) * posterior[i].weight;
    //     average_cos = cos(posterior[i].pose.theta) * posterior[i].weight;
    // }
    // double a_angle = atan2(average_sin, average_cos);
    // if(average_cos < 0) {
    //     a_angle += M_PI;
    // }
    // pose.theta = a_angle;
    return pose;
}
