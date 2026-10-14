# Triple Pendulum Lab

A calm, dark-mode simulator where a Deep Q-Network learns to balance a triple inverted pendulum on a cart.
One self-contained HTML file: vanilla JS, no build step needed to run it, no dependencies except Google Fonts.

Open `index.html` in a browser, or serve it with GitHub Pages (Settings, Pages, deploy from the main branch root).

## What is in it

- **Physics:** cart plus three hinged uniform rods, derived with Lagrangian mechanics on `q = [x, θ1, θ2, θ3]`
  (absolute angles, 0 is upright). Mass matrix from COM Jacobians, Coriolis/centripetal and gravity terms, Cholesky solve
  each step. RK4 at 5 ms sub-steps, 50 Hz zero-order-hold control, viscous cart friction and joint damping, a force
  limit, and inelastic end-stops on a ±2.4 m track (an impulse distributed through the mass matrix).
  The equations are commented in `src/core.js`.
- **Agent:** hand-written MLP 8 → 64 → 64 → 13 (ReLU, Adam, manual backprop). Double DQN, Huber loss, soft target updates,
  replay buffer, 13 force levels denser near zero.
- **Demonstration seeding (DQfD, on by default):** an internal controller designed in the browser from the current physics
  (linearise, discretise, solve the Riccati equation) records 60 noisy demo episodes labelled with the expert action.
  The network is pretrained for 1500 updates with a large-margin loss, then RL continues with demo samples mixed into
  each batch. The margin weight fades to 20% over the first 40k RL steps.
- **Training** runs in a background headless environment, time-sliced per animation frame. The visible pendulum is always
  the current greedy agent, running in real time.
- **Interaction:** drag the cart or any link (spring-damper through the Jacobian), arrow keys or A/D push the cart,
  Space pauses the simulation, R resets, Kick adds an impulse.

## Be realistic about training

A DQN trained from scratch usually will not master three links in minutes; without demonstrations it mostly learns to
delay the fall. With demonstrations it is much better, but results vary a lot by seed. In headless runs, pretraining alone
gave a greedy policy that balanced the full 500 steps in roughly 15 to 40% of episodes. Reinforcement learning after that was
sometimes a clear improvement and sometimes not within 120k steps. Treat any timing you see as a rough guide.

Training stops while the browser tab is hidden. Changing the physics sliders mid-training changes the environment under
the agent, so Reset Agent for a clean run.

## Develop

```
npm run build   # src/*  ->  index.html
npm test        # physics, expert and training checks (Node 18+, no dependencies)
```

`src/core.js` is DOM-free (physics, environment, expert, DQN, trainer, live simulation) and is what the tests import.
`src/ui.js` is the browser layer, `src/template.html` holds the markup and CSS.

## Push to GitHub

```
git init && git add . && git commit -m "Triple Pendulum Lab"
git branch -M main
git remote add origin git@github.com:<you>/triple-pendulum-lab.git
git push -u origin main
```

No licence is included; add one before publishing if you want others to reuse the code.
