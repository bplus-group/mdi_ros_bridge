# possible timebase types
uint16 TB_TYPE_TAI=0
uint16 TB_TYPE_WCD=1
uint16 TB_TYPE_UTC=2
uint16 TB_TYPE_LOCAL_NS=4097

# possible timebase flags
uint16 TB_FLAG_UNSYNCED=0
uint16 TB_FLAG_SYNCED=1

uint16 timebase_type
uint16 timebase_flag
uint64 timestamp