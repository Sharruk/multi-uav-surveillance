\# Target Benchmark Metrics \& STIRS-2025 Logs



The simulation must log the following metrics during evaluation to validate the algorithms and generate data for the STIRS Annexure A/B reports:



\## Algorithm Comparison Targets (Proposed vs MADDPG)

\* \*\*Swarm Success Rate (SSR):\*\* Target is $\\ge$ 85%. Must track if all drones complete surveillance without colliding.

\* \*\*Dynamic Adaptability (DA):\*\* Target is $\\le$ 0.12 s. Time taken to recalculate POMDP belief state when the Local Occupancy Grid changes due to occlusion.

\* \*\*Path Optimality (PO):\*\* Target is $\\le$ 1.1. 

\* \*\*Minimum Distance Margin (MDM):\*\* Maintain 1 to 5 meters between UAVs and static obstacles.



\## STIRS-2025 Relevance Outputs

\* \*\*Scalability Log:\*\* Record computation time (CT) as swarm size scales from 2 to 5 drones (Target CT $\\le$ 0.5 s).

\* \*\*Resilience Log:\*\* Track Target Recognition Rate (TRR) drop-off when artificial noise is injected into the Local Occupancy Grid (simulating SLAM drift/sensor failure).

