Header      HEADER
uint32      ID_SENDER
uint32      ID_RECEIVER
bool        INIT
bool        STEP

Mission[]   MISSION
Mission[]   ASSIGNED_MISSION

float64[]   X_POS
float64[]   Y_POS

uint32[]    CURR_VERTEX
uint32[]    NEXT_VERTEX

uint32[]    CURR_DST

uint32[]    CAPACITY
uint32[]    INIT_POS
time[]      MISSION_START_TIME
float32[]   MISSION_CURRENT_DISTANCE

int32[]      INTERFERENCE_STATUS
uint32[]    INTERFERENCE_COUNTER
uint32[]    MISSIONS_COMPLETED
uint32[]    TASKS_COMPLETED
float32[]   TOTAL_DISTANCE

uint32[]      GOAL_STATUS
Path[]      TRAILS
uint32      INIT_POS_INDEX

bool        END_SIMULATION

