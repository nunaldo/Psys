#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>
#include "utils.h"

enum DistributedAlgorithm : uint8_t {
    PRIMAL_DUAL = 0,
    ADMM,
    CONSENSUS
};

struct NeighborData {
    float u = 0.0f;
    float auxiliary = 0.0f;
};

class SuperController {
public:
    SuperController(uint8_t id,
                    float initial_cost,
                    float (&gain_matrix)[N_NODES][N_NODES],
                    float d0);

    void setAlgorithm(DistributedAlgorithm algo);
    void setNodeId(uint8_t id);
    void setFeedbackEnabled(bool enabled);
    void setCost(float cost);
    void setReference(float ref);
    void setBaseline(float baseline);

    float getDutyCycle() const;
    float getAuxiliaryData() const;
    float getFinalDutyCycle() const;
    float getCost() const;
    float getReference() const;
    DistributedAlgorithm getAlgorithm() const;
    bool isFeedbackEnabled() const;

    float runControlCycle(const NeighborData& node_j,
                          const NeighborData& node_k,
                          float measured_lux);

private:
    uint8_t my_id_;
    DistributedAlgorithm current_algo_;
    bool feedback_enabled_;

    float my_cost_;
    float my_L_ref_;
    float (&gain_matrix_)[N_NODES][N_NODES];
    uint8_t my_index_;
    uint8_t neighbor_j_index_;
    uint8_t neighbor_k_index_;
    float d0_i_;

    float feedforward_u_;
    float feedback_u_;
    float final_u_;

    float kp_;
    float ki_;
    float kd_;
    float integral_error_;
    float last_error_;

    float my_lambda_;
    float alpha_;
    float rho_;

    float copy_u_j_;
    float copy_u_k_;
    float y_ij_;
    float y_ik_;
    float rho_admm_;
    float admm_self_step_;
    float admm_copy_step_;
    float admm_violation_gain_;
    float admm_max_self_delta_;
    float admm_max_copy_delta_;
    bool admm_initialized_;

    float my_gradient_estimate_;
    float previous_local_gradient_;
    float consensus_step_;
    float consensus_penalty_gain_;
    float consensus_self_weight_;
    float consensus_neighbor_weight_;
    bool consensus_initialized_;

    static float clampUnit(float value);
    static float positivePart(float value);
    float selfGain() const;
    float gainFromNeighborJ() const;
    float gainFromNeighborK() const;
    float gainToNeighborJ() const;
    float gainToNeighborK() const;
    float computeEstimatedLux(const NeighborData& node_j,
                              const NeighborData& node_k) const;
    void resetOptimizationState();
    void runPrimalDual(const NeighborData& node_j, const NeighborData& node_k);
    void runADMM(const NeighborData& node_j, const NeighborData& node_k);
    void runConsensus(const NeighborData& node_j, const NeighborData& node_k);
};

#endif
