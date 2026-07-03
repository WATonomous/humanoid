import jax
from brax import envs
from brax.io import model, html
from brax.training.agents.ppo import networks as ppo_networks
from brax.training.acme import running_statistics

def main():
    print("1. Loading MJX Ant Environment...")
    env = envs.get_environment("ant", backend="mjx")
    
    print("2. Loading Trained Brain (ant_brain.pkl)...")
    # Load the saved parameters (normalizer_params, policy_params)
    params = model.load_params("ant_brain.pkl")

    print("3. Reconstructing Neural Network...")
    # Rebuild the exact PPO network architecture and normalizer used during training
    ppo_network = ppo_networks.make_ppo_networks(
        env.observation_size,
        env.action_size,
        preprocess_observations_fn=running_statistics.normalize
    )
    
    # Generate the inference function
    make_inference_fn = ppo_networks.make_inference_fn(ppo_network)
    
    # We set deterministic=True so the Ant uses its absolute best learned policy 
    # rather than randomly exploring like it did during training.
    inference_fn = jax.jit(make_inference_fn(params, deterministic=True))

    jit_reset = jax.jit(env.reset)
    jit_step = jax.jit(env.step)

    print("4. Simulating the Ant's movements (1,000 steps)...")
    rng = jax.random.PRNGKey(42)
    state = jit_reset(rng)
    
    rollout = []
    
    for _ in range(1000):
        rollout.append(state.pipeline_state)
        rng, act_rng = jax.random.split(rng)
        
        # The Brain decides the next action based on what it sees
        action, _ = inference_fn(state.obs, act_rng)
        
        # The Universe moves forward one tick
        state = jit_step(state, action)

    print("5. Rendering 3D Web Viewer...")
    html_path = "ant_playback.html"
    
    html.save(html_path, env.sys, rollout)
    
    print(f" SUCCESS! The 3D replay is saved to {html_path}")

if __name__ == "__main__":
    main()