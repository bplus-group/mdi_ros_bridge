# possible data types
uint16 DATA_TYPE_NOT_IMPLEMENTED_YET=0
uint16 DATA_TYPE_CSI2=18432
uint16 DATA_TYPE_I2C=16417
uint16 DATA_TYPE_CAN=16400
uint16 DATA_TYPE_JSON_STATUS=18512

# unique number for a data source
uint64 stream_id
# the id configured in a mdi device
uint8  device_instance
# the port numer of a mdi device where the data originated from
uint8  port_number
# a differentiation if the same port has multiple sources of the same data type
uint8  port_sub_index
# the data type
uint16 data_type
# a counter which is unique per stream_id
uint32 cycle_counter