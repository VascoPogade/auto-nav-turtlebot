# Branch used to navigate in a real life environment

### Set inital position
`rosrun auto_nav init_pose.py [init_pose yaml]`
- **init_pose yaml**: This should contain all mandatory information about the robots initial position 

*Example.yaml*
```yaml
position:
  x: [position x]
  y: [position y]

orientation:
  x: [orientation x]
  y: [orientation y]
  z: [orientation z]
  w: [orientation w]

covariance: [covariance (float array 32)]
```

### Navigate robot
`rosrun auto_nav goal_pose.py [pos_x] [pos_y] [ori_z] [ori_w`
- **pos_x:** initial x position
- **pos_y:** initial y position
- **ori_z:** initial z orientation
- **ori_w:** initial w orientation