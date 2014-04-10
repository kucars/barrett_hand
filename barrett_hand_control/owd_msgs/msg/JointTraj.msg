owd_msgs/Joints[] positions
float32[] blend_radius
uint32 options
string id

# options should be powers of 2, so that they can be OR'd together
uint32 opt_WaitForStart=1
uint32 opt_CancelOnStall=2
uint32 opt_CancelOnForceInput=4
uint32 opt_CancelOnTactileInput=8
uint32 opt_Synchronize=16
#uint32 opt_          =32  # placeholder for next value
