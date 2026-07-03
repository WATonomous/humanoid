import jax
import brax
from brax import envs
from brax.training.agents.ppo import train as ppo
from brax.io import model
import time

def main():
    # 1. Load the built-in MuJoCo Ant environment
    env_name = "ant"
    env = envs.get_environment(env_name, backend="mjx")

    # 2. Define the callback to print our progress to the terminal
    def progress_callback(num_steps, metrics):
        reward = metrics['eval/episode_reward']
        print(f"Step: {num_steps:,} | Average Reward: {reward:.2f}")

    print(f"Hardware detected: {jax.devices()[0]}")
    print("Spawning 1,024 parallel ants. Training for 5,000,000 steps...")
    
    start_time = time.time()

    # 3. Start the Reinforcement Learning Loop (PPO Algorithm)
    make_inference_fn, params, _ = ppo.train(
        environment=env,
        num_timesteps=5_000_000,
        num_evals=10,                      # How many times to print progress
        reward_scaling=0.1,
        episode_length=1000,
        normalize_observations=True,
        action_repeat=1,
        unroll_length=10,
        num_minibatches=32,
        num_updates_per_batch=4,
        discounting=0.99,
        learning_rate=3e-4,
        entropy_cost=1e-2,
        num_envs=1024,                     # 1,024 ants training at once!
        batch_size=512,
        seed=0,
        progress_fn=progress_callback      # Hook in our print function
    )

    # 4. Performance Metrics & Saving
    elapsed = time.time() - start_time
    print(f"\n Training completed in {elapsed:.2f} seconds!")

    # Save the physical "brain" (neural network weights) to your Windows folder
    brain_file = "ant_brain.pkl"
    model.save_params(brain_file, params)
    print(f" Neural Network saved as '{brain_file}'. Ready for playback!")

if __name__ == "__main__":
    main()