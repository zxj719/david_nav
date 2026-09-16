# ADR-0010: standalone Dock uses a dedicated default target topic

Status: implemented for P6.5 R8 / SPEC P4. Production is unchanged.

The retained standalone Dock shim previously loaded /goal_pose from its installed
robot-data file. That was the original node's default, but it also lets a Nav2
RViz target enter Dock independently. The precise launch already overrides the
topic to /dock_goal; standalone must default to that dedicated topic too.

Change only david_config/dock/standalone.yaml's goal_topic to /dock_goal. Keep
all sixteen parameter names and the other fifteen values, including historical
5-degree yaw/two retries and the user-requested 0.05 rad/s settling threshold.
No reusable-code robot literal is added. Explicit custom policy and ROS parameter
overrides keep precedence; this is a default repair, not a ban on custom topics.
Operators who explicitly select /goal_pose must still avoid competing consumers.

The no-argument dock_cli test follows actual default selection, config loading and
public parameter declaration using a fake node. It was red on /goal_pose and green
on /dock_goal. A separate comparison keeps the old frozen declaration reference
unchanged and allows exactly this one value difference. Existing ROS-override and
precise-launch checks protect the unchanged cases. Build and installed-data checks
verify the selected package policy, without initializing ROS or sending a goal.
