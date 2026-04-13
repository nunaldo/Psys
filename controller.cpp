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

float limitStep(float current, float candidate, float max_delta) {
    const float upper = current + max_delta;
    const float lower = current - max_delta;
    if (candidate > upper) {
        return upper;
    }
    if (candidate < lower) {
        return lower;
    }
    return candidate;
}

}  // namespace

SuperController::SuperController(uint8_t id,
                                 float initial_cost,
                                 float (&gain_matrix)[N_NODES][N_NODES],
                                 float d0)
    : my_id_(id),
      current_algo_(PRIMAL_DUAL),
      feedback_enabled_(false),
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
      kp_(0.12f),
      ki_(0.0015f),
      kd_(0.0f),
      integral_error_(0.0f),
      last_error_(0.0f),
      my_lambda_(0.0f),
      alpha_(0.01f),
      rho_(0.01f),
      copy_u_j_(0.0f),
      copy_u_k_(0.0f),
      y_ij_(0.0f),
      y_ik_(0.0f),
      rho_admm_(0.02f),
      admm_self_step_(0.003f),
      admm_copy_step_(0.005f),
      admm_violation_gain_(0.08f),
      admm_max_self_delta_(0.02f),
      admm_max_copy_delta_(0.03f),
      admm_initialized_(false),
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
    if (current_algo_ == ADMM) {
        return 0.5f * (y_ij_ + y_ik_);
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
            runPrimalDual(node_j, node_k, measured_lux);
            break;
        case ADMM:
            runADMM(node_j, node_k, measured_lux);
            break;
        case CONSENSUS:
            runConsensus(node_j, node_k, measured_lux);
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
        const float raw_error = expected_lux - measured_lux;
        const float feedback_deadband = 0.20f;
        float error = 0.0f;
        if (raw_error > feedback_deadband) {
            error = raw_error - feedback_deadband;
        } else if (raw_error < -feedback_deadband) {
            error = raw_error + feedback_deadband;
        }

        integral_error_ += error;
        if (integral_error_ > 10.0f) {
            integral_error_ = 10.0f;
        }
        if (integral_error_ < -10.0f) {
            integral_error_ = -10.0f;
        }

        const float derivative = error - last_error_;
        feedback_u_ = (kp_ * error) + (ki_ * integral_error_) + (kd_ * derivative);
        last_error_ = error;
    } else {
        feedback_u_ = 0.0f;
    }

    final_u_ = clampUnit(feedforward_u_ + feedback_u_);
    // The distributed optimizer must evolve around the duty that is actually
    // applied and broadcast, otherwise each node optimizes one value while the
    // network sees another and visible chatter appears.
    feedforward_u_ = final_u_;
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
    admm_initialized_ = false;
    my_gradient_estimate_ = 0.0f;
    previous_local_gradient_ = 0.0f;
    consensus_initialized_ = false;
}

void SuperController::runPrimalDual(const NeighborData& node_j,
                                    const NeighborData& node_k,
                                    float measured_lux) {
    const float visibility_error = my_L_ref_ - measured_lux;
    my_lambda_ += alpha_ * visibility_error;
    if (my_lambda_ < 0.0f) {
        my_lambda_ = 0.0f;
    }

    const float gradient = my_cost_ -
                           (my_lambda_ * selfGain()) -
                           (node_j.auxiliary * gainToNeighborJ()) -
                           (node_k.auxiliary * gainToNeighborK());
    feedforward_u_ = clampUnit(feedforward_u_ - (rho_ * gradient));
}

void SuperController::runADMM(const NeighborData& node_j,
                              const NeighborData& node_k,
                              float measured_lux) {
    // ADMM local model:
    // - keep the global decision u_i on this node
    // - keep local copies of the neighbors' duties
    // - penalize local visibility violation with the calibrated coupling model
    // - enforce copy_j = u_j and copy_k = u_k with scaled-dual ADMM updates
    if (!admm_initialized_) {
        copy_u_j_ = node_j.u;
        copy_u_k_ = node_k.u;
        admm_initialized_ = true;
    }

    const float lux_error = my_L_ref_ - measured_lux;
    const float deadband = 0.75f;

    float under_error = 0.0f;
    float over_error = 0.0f;
    if (lux_error > deadband) {
        under_error = lux_error - deadband;
    } else if (lux_error < -deadband) {
        over_error = (-lux_error) - deadband;
    }

    float gradient_u = my_cost_ + (0.02f * feedforward_u_);
    float gradient_copy_j = y_ij_ + rho_admm_ * (copy_u_j_ - node_j.u);
    float gradient_copy_k = y_ik_ + rho_admm_ * (copy_u_k_ - node_k.u);

    if (under_error > 0.0f) {
        const float penalty_scale = admm_violation_gain_ * under_error;
        gradient_u -= penalty_scale * selfGain();
        gradient_copy_j -= penalty_scale * gainFromNeighborJ();
        gradient_copy_k -= penalty_scale * gainFromNeighborK();
    } else if (over_error > 0.0f) {
        const float penalty_scale = admm_violation_gain_ * over_error;
        gradient_u += penalty_scale * selfGain();
        gradient_copy_j += penalty_scale * gainFromNeighborJ();
        gradient_copy_k += penalty_scale * gainFromNeighborK();
    } else {
        // Hold the operating point inside a small lux band to avoid visible
        // 0/1 chatter caused by model error and asynchronous neighbor updates.
        gradient_u = 0.0f;
    }

    const float next_u = clampUnit(feedforward_u_ - (admm_self_step_ * gradient_u));
    const float next_copy_j = clampUnit(copy_u_j_ - (admm_copy_step_ * gradient_copy_j));
    const float next_copy_k = clampUnit(copy_u_k_ - (admm_copy_step_ * gradient_copy_k));

    feedforward_u_ = limitStep(feedforward_u_, next_u, admm_max_self_delta_);
    copy_u_j_ = limitStep(copy_u_j_, next_copy_j, admm_max_copy_delta_);
    copy_u_k_ = limitStep(copy_u_k_, next_copy_k, admm_max_copy_delta_);

    y_ij_ += rho_admm_ * (copy_u_j_ - node_j.u);
    y_ik_ += rho_admm_ * (copy_u_k_ - node_k.u);
    if (y_ij_ > 0.5f) {
        y_ij_ = 0.5f;
    } else if (y_ij_ < -0.5f) {
        y_ij_ = -0.5f;
    }
    if (y_ik_ > 0.5f) {
        y_ik_ = 0.5f;
    } else if (y_ik_ < -0.5f) {
        y_ik_ = -0.5f;
    }
}

void SuperController::runConsensus(const NeighborData& node_j,
                                   const NeighborData& node_k,
                                   float measured_lux) {
    const float lux_error = my_L_ref_ - measured_lux;
    const float deadband = 0.75f;

    float under_error = 0.0f;
    float over_error = 0.0f;
    if (lux_error > deadband) {
        under_error = lux_error - deadband;
    } else if (lux_error < -deadband) {
        over_error = (-lux_error) - deadband;
    }

    float local_gradient = my_cost_ + (0.02f * feedforward_u_);
    if (under_error > 0.0f) {
        local_gradient -= consensus_penalty_gain_ * selfGain() * under_error;
    } else if (over_error > 0.0f) {
        local_gradient += consensus_penalty_gain_ * selfGain() * over_error;
    } else {
        // Hold duty nearly constant when measured lux is already inside a small
        // band around the target, otherwise consensus keeps drifting and flickers.
        local_gradient = 0.0f;
    }

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

    if (my_gradient_estimate_ > 40.0f) {
        my_gradient_estimate_ = 40.0f;
    } else if (my_gradient_estimate_ < -40.0f) {
        my_gradient_estimate_ = -40.0f;
    }

    previous_local_gradient_ = local_gradient;
    feedforward_u_ = clampUnit(feedforward_u_ - (consensus_step_ * my_gradient_estimate_));
}

