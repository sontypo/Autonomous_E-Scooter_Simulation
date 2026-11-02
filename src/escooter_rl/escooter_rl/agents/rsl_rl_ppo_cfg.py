# Copyright (c) 2026, Autonomous E-Scooter Simulation Project.
# All rights reserved.

from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg


@configclass
class EScooterGoalNavPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    """PPO Configuration for E-Scooter Goal Navigation & Self-Balancing."""

    num_steps_per_env = 24
    max_iterations = 1500
    save_interval = 50
    experiment_name = "escooter_goal_nav"
    empirical_normalization = False

    # Neural Network Architecture
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=1.0,
        actor_hidden_dims=[256, 128, 64],
        critic_hidden_dims=[256, 128, 64],
        activation="elu",
    )

    # PPO Algorithm Hyperparameters
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )


@configclass
class EScooterVelocityPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    """PPO Configuration for E-Scooter Velocity Tracking & Self-Balancing."""

    num_steps_per_env = 24
    max_iterations = 1200
    save_interval = 50
    experiment_name = "escooter_velocity"
    empirical_normalization = False

    policy = RslRlPpoActorCriticCfg(
        init_noise_std=1.0,
        actor_hidden_dims=[256, 128, 64],
        critic_hidden_dims=[256, 128, 64],
        activation="elu",
    )

    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )
