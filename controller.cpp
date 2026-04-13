#include "controller.h"

namespace {

float computeExpectedLux(float self_k,
                         float cross_kij,
                         float cross_kik,
                         float own_u,
                         float neighbor_j_u,
                         float neighbor_k_u,
                         float baseline) {
    return (self_k * own_u) +
           (cross_kij * neighbor_j_u) +
           (cross_kik * neighbor_k_u) +
           baseline;
}

}  // namespace

SuperController::SuperController(uint8_t id,
                                 float initial_cost,
                                 float (&gain_matrix)[N_NODES][N_NODES],
                                 float d0)
    : my_id_(id),
      current_algo_(PRIMAL_DUAL),
      feedback_enabled_(true),
      my_cost_(initial_cost),
      my_L_ref_(0.0f),
      gain_matrix_(gain_matrix),
      my_index_(0),
      neighbor_j_index_(1),
      neighbor_k_index_(2),
      d0_i_(d0),
      feedforward_u_(0.0f),
      feedback_u_(0.0f),
      final_u_(0.0f),
      kp_(0.5f),
      ki_(0.01f),
      kd_(0.05f),
      integral_error_(0.0f),
      last_error_(0.0f),
      my_lambda_(0.0f),
      alpha_(0.01f),
      rho_(0.01f),
      copy_u_j_(0.0f),
      copy_u_k_(0.0f),
      y_ij_(0.0f),
      y_ik_(0.0f),
      rho_admm_(0.05f),
      my_gradient_estimate_(0.0f),
      previous_local_gradient_(0.0f),
      consensus_step_(0.002f),
      consensus_penalty_gain_(0.05f),
      consensus_self_weight_(1.0f / 3.0f),
      consensus_neighbor_weight_(1.0f / 3.0f),
      consensus_initialized_(false) {
    setNodeId(id);
    resetOptimizationState();
}

void SuperController::setAlgorithm(DistributedAlgorithm algo) {
    current_algo_ = algo;
    resetOptimizationState();
}

void SuperController::setNodeId(uint8_t id) {
    my_id_ = id;
    if (id >= 1 && id <= N_NODES) {
        my_index_ = id - 1;
    } else if (id < N_NODES) {
        my_index_ = id;
    } else {
        my_index_ = 0;
    }

    neighbor_j_index_ = (my_index_ + 1) % N_NODES;
    neighbor_k_index_ = (my_index_ + 2) % N_NODES;
}

void SuperController::setFeedbackEnabled(bool enabled) {
    feedback_enabled_ = enabled;
    if (!feedback_enabled_) {
        feedback_u_ = 0.0f;
        integral_error_ = 0.0f;
        last_error_ = 0.0f;
    }
}

void SuperController::setCost(float cost) {
    my_cost_ = cost;
}

void SuperController::setReference(float ref) {
    my_L_ref_ = ref;
}

void SuperController::setBaseline(float baseline) {
    d0_i_ = baseline;
}

float SuperController::getDutyCycle() const {
    return feedforward_u_;
}

float SuperController::getAuxiliaryData() const {
    if (current_algo_ == PRIMAL_DUAL) {
        return my_lambda_;
    }
    if (current_algo_ == CONSENSUS) {
        return my_gradient_estimate_;
    }
    return 0.0f;
}

float SuperController::getFinalDutyCycle() const {
    return final_u_;
}

float SuperController::getCost() const {
    return my_cost_;
}

float SuperController::getReference() const {
    return my_L_ref_;
}

DistributedAlgorithm SuperController::getAlgorithm() const {
    return current_algo_;
}

bool SuperController::isFeedbackEnabled() const {
    return feedback_enabled_;
}

float SuperController::runControlCycle(const NeighborData& node_j,
                                       const NeighborData& node_k,
                                       float measured_lux) {
    switch (current_algo_) {
        case PRIMAL_DUAL:
            runPrimalDual(node_j, node_k);
            break;
        case ADMM:
            runADMM(node_j, node_k);
            break;
        case CONSENSUS:
            runConsensus(node_j, node_k);
            break;
    }

    if (feedback_enabled_) {
        const float expected_lux = computeExpectedLux(
            selfGain(),
            gainFromNeighborJ(),
            gainFromNeighborK(),
            feedforward_u_,
            node_j.u,
            node_k.u,
            d0_i_);
        const float error = expected_lux - measured_lux;

        integral_error_ += error;
        if (integral_error_ > 50.0f) {
            integral_error_ = 50.0f;
        }
        if (integral_error_ < -50.0f) {
            integral_error_ = -50.0f;
        }

        const float derivative = error - last_error_;
        feedback_u_ = (kp_ * error) + (ki_ * integral_error_) + (kd_ * derivative);
        last_error_ = error;
    } else {
        feedback_u_ = 0.0f;
    }

    final_u_ = clampUnit(feedforward_u_ + feedback_u_);
    return final_u_;
}

float SuperController::clampUnit(float value) {
    if (value > 1.0f) {
        return 1.0f;
    }
    if (value < 0.0f) {
        return 0.0f;
    }
    return value;
}

float SuperController::positivePart(float value) {
    return value > 0.0f ? value : 0.0f;
}

float SuperController::selfGain() const {
    return gain_matrix_[my_index_][my_index_];
}

float SuperController::gainFromNeighborJ() const {
    return gain_matrix_[my_index_][neighbor_j_index_];
}

float SuperController::gainFromNeighborK() const {
    return gain_matrix_[my_index_][neighbor_k_index_];
}

float SuperController::gainToNeighborJ() const {
    return gain_matrix_[neighbor_j_index_][my_index_];
}

float SuperController::gainToNeighborK() const {
    return gain_matrix_[neighbor_k_index_][my_index_];
}

float SuperController::computeEstimatedLux(const NeighborData& node_j,
                                           const NeighborData& node_k) const {
    return computeExpectedLux(
        selfGain(),
        gainFromNeighborJ(),
        gainFromNeighborK(),
        feedforward_u_,
        node_j.u,
        node_k.u,
        d0_i_);
}

void SuperController::resetOptimizationState() {
    my_lambda_ = 0.0f;
    copy_u_j_ = 0.0f;
    copy_u_k_ = 0.0f;
    y_ij_ = 0.0f;
    y_ik_ = 0.0f;
    my_gradient_estimate_ = 0.0f;
    previous_local_gradient_ = 0.0f;
    consensus_initialized_ = false;
}

void SuperController::runPrimalDual(const NeighborData& node_j, const NeighborData& node_k) {
    const float estimated_l = computeEstimatedLux(node_j, node_k);

    my_lambda_ += alpha_ * (my_L_ref_ - estimated_l);
    if (my_lambda_ < 0.0f) {
        my_lambda_ = 0.0f;
    }

    const float gradient = my_cost_ -
                           (my_lambda_ * selfGain()) -
                           (node_j.auxiliary * gainToNeighborJ()) -
                           (node_k.auxiliary * gainToNeighborK());
    feedforward_u_ = clampUnit(feedforward_u_ - (rho_ * gradient));
}

void SuperController::runADMM(const NeighborData& node_j, const NeighborData& node_k) {
    y_ij_ += rho_admm_ * (copy_u_j_ - node_j.u);
    y_ik_ += rho_admm_ * (copy_u_k_ - node_k.u);

    const float residual_j = copy_u_j_ - node_j.u;
    const float residual_k = copy_u_k_ - node_k.u;

    const float gradient_u = my_cost_ +
                             rho_admm_ * (feedforward_u_ - copy_u_j_) +
                             rho_admm_ * (feedforward_u_ - copy_u_k_);
    const float gradient_copy_j = y_ij_ + rho_admm_ * residual_j;
    const float gradient_copy_k = y_ik_ + rho_admm_ * residual_k;

    feedforward_u_ = clampUnit(feedforward_u_ - (0.01f * gradient_u));
    copy_u_j_ = clampUnit(copy_u_j_ - (0.01f * gradient_copy_j));
    copy_u_k_ = clampUnit(copy_u_k_ - (0.01f * gradient_copy_k));
}

void SuperController::runConsensus(const NeighborData& node_j, const NeighborData& node_k) {
    const float estimated_l = computeEstimatedLux(node_j, node_k);
    const float visibility_violation = positivePart(my_L_ref_ - estimated_l);
    const float local_gradient =
        my_cost_ - (consensus_penalty_gain_ * selfGain() * visibility_violation);

    if (!consensus_initialized_) {
        my_gradient_estimate_ = local_gradient;
        consensus_initialized_ = true;
    } else {
        const float mixed_estimate =
            (consensus_self_weight_ * my_gradient_estimate_) +
            (consensus_neighbor_weight_ * node_j.auxiliary) +
            (consensus_neighbor_weight_ * node_k.auxiliary);
        my_gradient_estimate_ = mixed_estimate + (local_gradient - previous_local_gradient_);
    }

    previous_local_gradient_ = local_gradient;
    feedforward_u_ = clampUnit(feedforward_u_ - (consensus_step_ * my_gradient_estimate_));
}

