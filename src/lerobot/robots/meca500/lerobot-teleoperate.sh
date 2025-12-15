lerobot-teleoperate `
   --robot.type=meca500 `
   --robot.ip_address=192.168.0.100 `
   --robot.id=my_meca500 `
   --teleop.type=meca500_bota `
   --teleop.id=my_teleop

lerobot-record `
    --robot.type=meca500 `
    --robot.ip_address=192.168.0.100`
    --dataset.repo_id=$AdamAxelrod/meca500-test-dataset`
    --dataset.episode_time_s=30`
    --dataset.num_episodes=3`
    --dataset.single_task="test1"