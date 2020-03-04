Header      header
bool        PICKUP
uint32      ID
# usato da OnlineTaskPlanner e OnlineAgent
# per identificare a quale finestra appartiene il task
uint32      WINDOW_ID
uint32      PRIORITY
uint32[]    ITEM
# approximante route used by the task planner to evaluate partitions
uint32[]    ROUTE
# contains the destinations vertices that must be reached through the route
# in the current implementation pickup and home vertices are known by the agents
# and must not be specified
uint32[]    DSTS
# contains the demands of each task, used by task planner to aggregate
uint32[]    DEMANDS
float64     V
uint32      TOT_DEMAND
uint32      PATH_DISTANCE

uint32      GOOD