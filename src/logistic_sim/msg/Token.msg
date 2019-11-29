Header      HEADER
uint32      ID_SENDER
uint32      ID_RECEIVER
# flag di stato
# TODO: unire in un solo campo uint32
bool        INIT
bool        ALLOCATE
bool        CHECK   # controllo validità percorsi
bool        CALCULATE_PATHS
bool        NEW_MISSIONS_AVAILABLE
bool        GOOD_PATHS  # percorsi contenuti nel token validi

Mission[]   MISSION
int32[]     TAKEN_MISSION   # usato da CFree per tenere traccia dell'assegnamento missioni

# posizioni dei robot nella mappa
float64[]   X_POS
float64[]   Y_POS

# vertici attuali e successivi dei robot
uint32[]    CURR_VERTEX
uint32[]    NEXT_VERTEX

# destinazione corrente di ogni robot
uint32[]    CURR_DST

# capacità di carico di ogni robot
uint32[]    CAPACITY

# vertici iniziali dei robot
uint32[]    INIT_POS

# statistiche varie
time[]      MISSION_START_TIME
float32[]   MISSION_CURRENT_DISTANCE
int32[]     INTERFERENCE_STATUS
uint32[]    INTERFERENCE_COUNTER
uint32[]    MISSIONS_COMPLETED
uint32[]    TASKS_COMPLETED
float32[]   TOTAL_DISTANCE

# campi per GlobalAgent
uint32[]    GOAL_STATUS
Path[]      TRAILS
uint32      INIT_POS_INDEX
bool[]      REACHED_HOME
int32       ACTIVE_ROBOTS

# flag per terminazione simulazione
bool        END_SIMULATION

