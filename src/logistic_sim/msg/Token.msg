Header      HEADER
uint32      ID_SENDER
uint32      ID_RECEIVER
# flag di stato
# TODO: unire in un solo campo uint32
bool        INIT
bool        ALLOCATE
bool        CHECK   # controllo validità percorsi
bool        CALCULATE_PATHS
bool        NEW_MISSIONS_AVAILABLE  # indica agli agenti che sono disponibili nuove missioni nel token
bool        GOOD_PATHS  # percorsi contenuti nel token validi
bool        SHUTDOWN    # termina simulazione (per ora usato solo da Online)
bool        ALL_MISSIONS_INSERTED   # attivato dal taskplanner quando tutte le missioni sono nel token

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
Path[]      NEW_TRAILS
Path[]      HOME_TRAILS # per separare ritorno a casa dal percorso dei task
uint32      INIT_POS_INDEX
bool[]      REACHED_HOME
int32       ACTIVE_ROBOTS

# flag per terminazione simulazione
bool        END_SIMULATION

