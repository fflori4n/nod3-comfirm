#include <string>
#include <cstdint>
#include "esp_err.h"

/* macro for printing timestamps for debugging, kind of sketchy*/
#define PRINT_TMSTAMP(T) \
    (uint16_t)((T) / (365 * 24 * 3600) + 1970),                        /* years */ \
    (uint16_t)(((T) % (365 * 24 * 3600)) / (30 * 24 * 3600)),          /* months */ \
    (uint16_t)(((T) % (30 * 24 * 3600)) / (24 * 3600)),                /* days */ \
    (uint16_t)(((T) % (24 * 3600)) / 3600),                            /* hours */ \
    (uint16_t)(((T) % 3600) / 60),                                     /* minutes */ \
    (uint16_t)((T) % 60)                                               /* seconds */

constexpr static char* log_tmmeas{COLOR_BLUE"TMMEAS"COLOR_WHITE};

enum class Log_status_t : uint8_t {
    UNPROCESSED  = 0b000,
    RDY_TO_SEND  = 0b001,
    IN_PROGRESS  = 0b010,
    SEND_RETRY   = 0b011,
    SEND_SUCCESS = 0b100,
    SEND_FAIL    = 0b101,
    DISCARD      = 0b110
};

char log_status_str[][15] = {"UNPROCESSED", "RDY_TO_SEND", "IN_PROGRESS", "SEND_RETRY", "SEND_SUCCESS", "SEND_FAIL", "DISCARD"};

enum class Data_encoding_t : uint8_t{
    BINARY     = 0,
    UINT       = 1,
    INT        = 2,
    FLOAT      = 3,
    TEXTLOOKUP = 4,
    _UNDEF     = 5
};

/* NOTE: this impelementation will support 127 different types */
enum class Measurement_type : uint8_t {
    ERROR = 0x00,   /* Error states: */
    SLOT_FREE = 1u, /* Memory slot is free to use */
    /* Open measurement types: */
    MCU_TMRST,
    MCU_TEMP,
    MCU_RSTVCT,
    MCU_SOL,
    MCU_MINMEM,
    MCU_RSTCNT,
    MCU_WAKCNT,
    PWR_UPTIME,
    PWR_RECUPT,
    PWR_UPTTTL,
    PWR_RECTTL,
    PWR_BATV
};

class Measurement_entity_type
{   
private:
    

public:
    static constexpr uint16_t tag_buff_size{20};
    Measurement_type measurement_type;
    char tag[tag_buff_size];
    Data_encoding_t encoding;
    uint32_t range_max;
    uint32_t range_min;
    char *text_lookup;
    uint32_t measure_period_min;
    uint32_t report_period_min;

    Measurement_entity_type() = default;
    Measurement_entity_type(const Measurement_type meas_type , const char *tag_ptr, Data_encoding_t enc, uint32_t min, uint32_t max, char *value_str_lookup, uint16_t measure_p, uint16_t send_p)
        : measurement_type(meas_type), encoding(enc), range_max(max), range_min(min), text_lookup(value_str_lookup), measure_period_min(measure_p), report_period_min(send_p)
    {
        int16_t num_inbytes = std::snprintf(tag, tag_buff_size, "%s", tag_ptr);
        if (num_inbytes < 0)
        {
            ESP_LOGW(log_tmmeas, "Unexpected ER");
        }
        else if (num_inbytes >= tag_buff_size)
        {
            ESP_LOGW(log_tmmeas, "Tag was truncated");
        }
    }

    void dbg_print() const {
        ESP_LOGI(log_tmmeas, " %s - encoding: %d, [%lld:%lld], Measures every: %lld, Reports every: %lld", tag, static_cast<uint32_t>(encoding), range_min, range_max, measure_period_min, report_period_min);
        return;
    }
};


Measurement_entity_type measurement_types[] = {
    {Measurement_type::ERROR, "ER",     Data_encoding_t::FLOAT, 0, 0xFFFF,         nullptr, 5,  15},
    {Measurement_type::SLOT_FREE, "FREE",     Data_encoding_t::FLOAT, 0, 0xFFFF,         nullptr, 5,  15},
    {Measurement_type::MCU_TMRST,   "mcu_tmrst",     Data_encoding_t::FLOAT,         0, 0xFFFF,         nullptr, 5,  15},
    {Measurement_type::MCU_TEMP,    "mcu_tmp",       Data_encoding_t::FLOAT,         0, 0xFFFF,         nullptr, 5,  15},
    {Measurement_type::MCU_RSTVCT,  "mcu_rstvct",    Data_encoding_t::TEXTLOOKUP,    0, 0,              nullptr, 1,  0},
    {Measurement_type::MCU_SOL,     "mcu_sol",       Data_encoding_t::UINT,          0, 0,              nullptr, 15, 15},
    {Measurement_type::MCU_MINMEM,  "mcu_minmem",    Data_encoding_t::UINT,          0, 0,              nullptr, 15, 15},
    {Measurement_type::MCU_RSTCNT,  "mcu_rstid",     Data_encoding_t::UINT,          0, 0,              nullptr, 15, 15},
    {Measurement_type::MCU_WAKCNT,  "mcu_wknum",     Data_encoding_t::UINT,          0, 0,              nullptr, 15, 15},

    {Measurement_type::PWR_UPTIME,  "mcu_upboot",           Data_encoding_t::FLOAT,          0, 0,              nullptr, 15, 15},
    {Measurement_type::PWR_RECUPT,  "mcu_upbootrecord",     Data_encoding_t::FLOAT,          0, 0,              nullptr, 15, 15},
    {Measurement_type::PWR_UPTTTL,  "mcu_uptotal",          Data_encoding_t::FLOAT,          0, 0,              nullptr, 15, 15},
    {Measurement_type::PWR_RECTTL,  "mcu_sleepratio",       Data_encoding_t::FLOAT,          0, 0,              nullptr, 15, 15},
    {Measurement_type::PWR_BATV,    "mcu_batv",             Data_encoding_t::FLOAT,          0, 0,              nullptr, 15, 15},
};

class Timestamped_measurement_storage;

class Timestamped_measurement{

    private:
    public:

        static constexpr uint16_t keyval_buffer_size{35};

        typedef union metadata_t{
            uint32_t u32;
            struct {
                Measurement_type measurement_type    : 8;
                uint16_t time_offset_secs            : 16;
                uint8_t is_initialized               : 1;
                Log_status_t log_status              : 4;
                uint8_t er_logged                    : 2;
                uint8_t reserved_bits                : 1;
            };
        } metadata_t;

        metadata_t measurement_metadata;
        uint32_t data{0u};

        static constexpr uint8_t value_str_buffer_size{25};

        Timestamped_measurement(){
            this->measurement_metadata.measurement_type = Measurement_type::SLOT_FREE;
            this->measurement_metadata.time_offset_secs = 0xFFFF;
            this->measurement_metadata.is_initialized = 0u;
            this->measurement_metadata.log_status = Log_status_t::UNPROCESSED;
            this->measurement_metadata.reserved_bits = 0u;
            this->data = 0x0000;
        };

        Timestamped_measurement(Measurement_type measurement_type_code, uint32_t data){
            this->measurement_metadata.measurement_type = measurement_type_code;
            this->measurement_metadata.time_offset_secs = 0xFFFF;  /* Signal that time is not set for this measurement */
            this->measurement_metadata.is_initialized = 1u;
            this->measurement_metadata.log_status = Log_status_t::UNPROCESSED;
            this->measurement_metadata.reserved_bits = 0u;
            this->data = data;
        }

        /* Put conversion here, hopefully cleaner than random static cast at random places, float turned out to not behave well with static_cast */
        /* NOTE: did not do template to be more explicit and not sure how diffrent types w diff lengths would behave */
        Timestamped_measurement(Measurement_type measurement_type_code, int32_t data) : Timestamped_measurement(measurement_type_code, static_cast<uint32_t>(data)){}
        /*Timestamped_measurement(Measurement_type measurement_type_code, float data) : Timestamped_measurement(measurement_type_code, std::bit_cast<uint32_t>(data)){}*/

        void clear_slot(){
            this->measurement_metadata.measurement_type = Measurement_type::SLOT_FREE;
            this->measurement_metadata.time_offset_secs = 0xFFFF;
            this->measurement_metadata.is_initialized = 0u;
            this->measurement_metadata.log_status = Log_status_t::UNPROCESSED;
            this->measurement_metadata.reserved_bits = 0u;
            this->data = 0x0000;
        }

        //constexpr bool operator==(const Timestamped_measurement& other) const = default;
        bool operator==(const Timestamped_measurement& other) const {
            return ((this->measurement_metadata.u32 == other.measurement_metadata.u32) && (this->data == other.data));
        }

        static Data_encoding_t get_encoding(auto sensor){
            for(const Measurement_entity_type& measurement_ent_type : measurement_types){
                if(sensor == measurement_ent_type.measurement_type){
                    return measurement_ent_type.encoding;
                }
            }
            return Data_encoding_t::_UNDEF;
        }

        static const Measurement_entity_type& get_entity_type(auto sensor){

            for(const Measurement_entity_type& measurement_ent_type : measurement_types){
                if(sensor == measurement_ent_type.measurement_type){
                    return measurement_ent_type;
                }
            }

            return measurement_types[0];
        }

        void print(const time_t& base_timestamp, const time_t& now_unix_timestamp= 0)
        {
            Timestamped_measurement& measurement = *this;
            Data_encoding_t encoding = Timestamped_measurement::get_encoding(this->measurement_metadata.measurement_type);

            auto get_time_str = [](time_t unix_time){

                constexpr uint16_t sizeof_timestrbuffer{32};
                std::array<char, sizeof_timestrbuffer> timestamp_string{};
                std::tm timestamp_tm = *std::localtime(&unix_time);
                std::strftime(&(timestamp_string[0]), sizeof_timestrbuffer, "%Y-%m-%dT%H:%M:%S%z", &timestamp_tm);
                return timestamp_string;
            };


            auto get_time_diff_str = [base_timestamp, now_unix_timestamp, &measurement](void){

                const time_t seconds_diff = (now_unix_timestamp - base_timestamp - measurement.measurement_metadata.time_offset_secs);
                const uint32_t hours   = (seconds_diff / 3600);
                const uint32_t minutes = (seconds_diff % 3600) / 60;
                const uint32_t seconds = (seconds_diff % 60);

                constexpr uint16_t sizeof_timestrbuffer{32};
                std::array<char, sizeof_timestrbuffer> timestamp_string{};
                snprintf(&(timestamp_string[0]), sizeof_timestrbuffer, "%02ld:%02ld:%02ld", hours, minutes, seconds);
                return timestamp_string;
            };

            switch (encoding)
            {
            case Data_encoding_t::BINARY:
                ESP_LOGI("MEASUREMENT", "value: %s type: BIN, status: %s taken at: %s (offs: %ld - %s ago)", (measurement.data == 0u ? "False" : "True"), log_status_str[static_cast<uint8_t>(measurement.measurement_metadata.log_status)], get_time_str(base_timestamp + measurement.measurement_metadata.time_offset_secs), measurement.measurement_metadata.time_offset_secs, get_time_diff_str());
                break;
            case Data_encoding_t::UINT:
                ESP_LOGI("MEASUREMENT", "value: %ld type: UINT32, status: %s taken at: %s (offs: %ld - %s ago)", static_cast<uint32_t>(measurement.data), log_status_str[static_cast<uint8_t>(measurement.measurement_metadata.log_status)], get_time_str(base_timestamp + measurement.measurement_metadata.time_offset_secs), measurement.measurement_metadata.time_offset_secs, get_time_diff_str());
                break;
            case Data_encoding_t::INT:
                ESP_LOGI("MEASUREMENT", "value: %ld type: INT32, status: %s taken at: %s (offs: %ld - %s ago)", static_cast<int32_t>(measurement.data), log_status_str[static_cast<uint8_t>(measurement.measurement_metadata.log_status)], get_time_str(base_timestamp + measurement.measurement_metadata.time_offset_secs), measurement.measurement_metadata.time_offset_secs, get_time_diff_str());
                break;
            case Data_encoding_t::FLOAT:
                /*ESP_LOGI("MEASUREMENT", "value: %0.2f type: FLOAT, status: %s taken at: %s (offs: %ld - %s ago)", std::bit_cast<float>(measurement.data), log_status_str[static_cast<uint8_t>(measurement.measurement_metadata.log_status)], get_time_str(base_timestamp + measurement.measurement_metadata.time_offset_secs), measurement.measurement_metadata.time_offset_secs, get_time_diff_str());*/
                break;

            default:
                break;
            }
        }

        void get_service_data_keyval(char (&keyval_buffer)[50]){

            const Timestamped_measurement& measurement = *this;
            const Measurement_entity_type& sensor_type = Timestamped_measurement::get_entity_type(this->measurement_metadata.measurement_type);
            if(sensor_type.measurement_type <= Measurement_type::SLOT_FREE){
                return;
            }

            std::array<char, value_str_buffer_size> value_buffer{};

            switch (sensor_type.encoding)
            {
                case Data_encoding_t::BINARY :
                    snprintf(&(value_buffer[0]), value_str_buffer_size, "%d", measurement.data == 0u ? 0 : 1);
                    break;
                case Data_encoding_t::UINT :
                    snprintf(&(value_buffer[0]), value_str_buffer_size, "%ld", static_cast<uint32_t>(measurement.data));
                    break;
                case Data_encoding_t::INT :
                    snprintf(&(value_buffer[0]), value_str_buffer_size, "%ld", static_cast<int32_t>(measurement.data));
                    break;
                case Data_encoding_t::FLOAT :
                    /*snprintf(&(value_buffer[0]), value_str_buffer_size, "%0.2f", (std::bit_cast<float>(measurement.data)));*/
                    break;
                default:
                    snprintf(&(value_buffer[0]), value_str_buffer_size, "None");
                    break;
            }

            snprintf(keyval_buffer, 50, "\"%s\":%s,", sensor_type.tag, &value_buffer[0]);
            return;
        }
};

class Timestamped_measurement_storage{

    public:
        static constexpr time_t Undefined_base_timestamp{0xFFFFFFFF};
        static constexpr uint16_t numof_measurement_slots{40}; /*100*/
        inline static time_t base_timestamp{Undefined_base_timestamp};
        inline static Timestamped_measurement tm_measurement_list[numof_measurement_slots];

    public:
        Timestamped_measurement_storage() = default;

        esp_err_t set_base_timestamp(const time_t& new_base_timestamp){
            return ESP_OK;
        }

        static std::tuple<uint16_t, uint16_t> get_minmax_base_offset_secs(time_t& unix_period_start, const uint16_t& period_duration){

            uint16_t offset_start = (unix_period_start - base_timestamp) > 0 ? (unix_period_start - base_timestamp) : 0u;
            uint16_t offset_end = ((unix_period_start + period_duration) - base_timestamp) > 0 ? ((unix_period_start + period_duration) - base_timestamp) : 0u;

            return {offset_start, offset_end};
        }

        /* NOTE: std::expected?? */
        static std::tuple<esp_err_t, uint16_t> get_time_offset(){

            auto [mcu_time_unix, time_reliability, now_tm] = Ntp_time::evaluate_mcu_rtc(true);

            if((time_reliability != Ntp_time::rtc_time_sts_t::OK) && (time_reliability != Ntp_time::rtc_time_sts_t::Not_synced_usable)){
                return {ESP_FAIL, 0xFFFF};
            }

            /* Next check if base timestamp was set. */
            if(Undefined_base_timestamp == base_timestamp){
                base_timestamp = mcu_time_unix;
                ESP_LOGI("TM_MEAS"," Base timestamp was set for the first time as %lllld", base_timestamp);
            }
            else{
                if((mcu_time_unix - base_timestamp) > 0xFFFF){
                    ESP_LOGW("TM_MEAS"," Base timestamp is too. measurement offset is overflowing.");
                    return {ESP_FAIL, 0xFFFF};
                }
            }
            return {ESP_OK, (mcu_time_unix - base_timestamp)};
        }

        static Data_encoding_t get_encoding(auto sensor){
            for(const Measurement_entity_type& measurement_ent_type : measurement_types){
                if(sensor == measurement_ent_type.measurement_type){
                    return measurement_ent_type.encoding;
                }
            }
            return Data_encoding_t::_UNDEF;
        }

        static esp_err_t get_tag_label(auto sensor, char (&tag_buff) [20]){
            for(const Measurement_entity_type& measurement_ent_type : measurement_types){
                if(sensor == measurement_ent_type.measurement_type){
                    snprintf(tag_buff, 20, "%s", measurement_ent_type.tag);
                    return ESP_OK;
                }
            }
            return ESP_FAIL;
        }

        static bool is_duplicate(Timestamped_measurement& new_measurement){
            return (std::find_if(std::begin(tm_measurement_list), std::end(tm_measurement_list), [&](Timestamped_measurement& measurement){
                return new_measurement == measurement; }) != std::end(tm_measurement_list));
        }

        static bool is_duplicate_timestamp(const Timestamped_measurement& new_measurement){
            return (std::find_if(std::begin(tm_measurement_list), std::end(tm_measurement_list), [&](Timestamped_measurement& measurement){
                return measurement.measurement_metadata.measurement_type > Measurement_type::SLOT_FREE &&
                new_measurement.measurement_metadata.measurement_type > Measurement_type::SLOT_FREE &&
                new_measurement.measurement_metadata.time_offset_secs == measurement.measurement_metadata.time_offset_secs &&
                new_measurement.measurement_metadata.measurement_type == measurement.measurement_metadata.measurement_type;
            }) != std::end(tm_measurement_list));
        }

        static esp_err_t compact_storage(uint32_t numof_empty_required){
            /* This will take a long time to run, but needs to be done. Drop samples to make space in buffer. */

            /* Take time range min/max */
            /* group into bins, new, middle, old */

            std::sort(std::begin(tm_measurement_list), std::end(tm_measurement_list),[](const Timestamped_measurement& meas1, const Timestamped_measurement& meas2) {
                if(meas1.measurement_metadata.measurement_type == meas2.measurement_metadata.measurement_type){
                    return meas1.measurement_metadata.time_offset_secs < meas2.measurement_metadata.time_offset_secs;
                }
                return meas1.measurement_metadata.measurement_type < meas2.measurement_metadata.measurement_type;
            });

            uint16_t numof_freed_slots = 0;
            for(Measurement_entity_type& measurement_type : measurement_types)
            {
                /* We have selected a measurement type to work with*/

                /* Now count the number of objects of this type */
                uint16_t numof_type_measurement = std::count_if(std::begin(tm_measurement_list), std::end(tm_measurement_list), [&](Timestamped_measurement& measurement){ 
                    if(measurement.measurement_metadata.measurement_type <= Measurement_type::SLOT_FREE){
                        /* Either free or error, do not count */
                        return false;
                    }

                     return measurement.measurement_metadata.measurement_type == measurement_type.measurement_type; 
                });

                ESP_LOGI("TM_MEAS","Compactor pruning: %s, numof measurements: %ld", measurement_type.tag, numof_type_measurement);

                uint16_t measurement_series_index = 0;
                uint16_t subdivision_counter = 0;
                for(Timestamped_measurement& selected_measurement : tm_measurement_list){

                    /* check if type matches (at the same time this will make sure slot is not free or error)*/
                    if(selected_measurement.measurement_metadata.measurement_type != measurement_type.measurement_type){
                        continue;
                    }

                    if(measurement_series_index <= (0.33 * numof_type_measurement)){
                        if(subdivision_counter == 2){
                            selected_measurement.clear_slot();
                            numof_freed_slots++;
                        }
                    }
                    else if(measurement_series_index <= (0.66 * numof_type_measurement)){
                        if(subdivision_counter == 0 || subdivision_counter == 2){
                            selected_measurement.clear_slot();
                            numof_freed_slots++;
                        }
                    }
                    else
                    {
                        if(subdivision_counter == 0 || subdivision_counter == 1 || subdivision_counter == 2){
                            selected_measurement.clear_slot();
                            numof_freed_slots++;
                        }
                    }

                    measurement_series_index++;
                    subdivision_counter = (subdivision_counter + 1) % 4;
                }
            }

            ESP_LOGI("TM_MEAS","Compactor finished. Additional %ld slot were emptied.", numof_freed_slots);

            return ESP_OK;
        }

        inline esp_err_t insert_measurement(Timestamped_measurement new_measurement){

            ESP_LOGW("TM_MEAS","looking to insert:");
            new_measurement.print(0);

            if(new_measurement.measurement_metadata.measurement_type <= Measurement_type::SLOT_FREE || new_measurement.measurement_metadata.is_initialized == 0u){
                ESP_LOGW("TM_MEAS","The measurement object is invalid, will not be added.");
                return ESP_FAIL;
            }

            if(new_measurement.measurement_metadata.time_offset_secs == 0xFFFF)
            {
                ESP_LOGI("TM_MEAS","setting timeoffset.");
                auto [time_offs_set_result, tm_offset_secs] = Timestamped_measurement_storage::get_time_offset();

                /* TODO: NOTE: Edge case, when system time is not available, this will be set as 0xFFFF, I don't know what to do with these measurements for now. Send it as up to date? ignore it? send as oldest? */
                new_measurement.measurement_metadata.time_offset_secs = tm_offset_secs;

                if(time_offs_set_result == ESP_OK){
                    ESP_LOGW("TM_MEAS","failed to set time offset.");
                }
            }


            /* Check once more, Avoid adding the same measurement twice */
            /* NOTE: this is a little bit of overhead, probably can be skipped, or just test if timestamp is invalid and don't add those. Or just add it and don't care, or optimize to loop only once when checking for free slot*/
            if(true == Timestamped_measurement_storage::is_duplicate_timestamp(new_measurement)){
                ESP_LOGW("TM_MEAS","Measurement is a duplicate, skipped inserting.");
                return ESP_FAIL;
            }

            /* TODO: validate measurement */
            ESP_LOGI("TM_MEAS","Trying to insert new measurement %lld", new_measurement.data);
            auto found_slot = std::find_if(std::begin(tm_measurement_list), std::end(tm_measurement_list), [](const Timestamped_measurement& slot){
                return slot.measurement_metadata.measurement_type == Measurement_type::SLOT_FREE;
            });

            if(found_slot == std::end(tm_measurement_list)){
                ESP_LOGW("TM_MEAS","Measurement storage is full. Will run compactor.");
                Timestamped_measurement_storage::compact_storage(1);
                /*return ESP_FAIL;*/
            }

            (*found_slot) = new_measurement;
            return ESP_OK;
        };
    
        inline void print_measurements(void) {

            auto [mcu_time_unix, time_reliability, now_tm] = Ntp_time::evaluate_mcu_rtc(true);

            std::sort(std::begin(tm_measurement_list), std::end(tm_measurement_list),[](const Timestamped_measurement& meas1, const Timestamped_measurement& meas2) {

                if(meas1.measurement_metadata.measurement_type == meas2.measurement_metadata.measurement_type){
                    return meas1.measurement_metadata.time_offset_secs < meas2.measurement_metadata.time_offset_secs;
                }
                
                return meas1.measurement_metadata.measurement_type < meas2.measurement_metadata.measurement_type;
            });

            uint16_t numof_free_slots{0};
            for(Measurement_entity_type measurement_type : measurement_types){

                ESP_LOGI("TM_MEAS","\t[%s] :", measurement_type.tag);

                std::for_each(std::begin(Timestamped_measurement_storage::tm_measurement_list), std::end(Timestamped_measurement_storage::tm_measurement_list),[&](Timestamped_measurement& measurement)
                {
                    if (Measurement_type::SLOT_FREE >= measurement.measurement_metadata.measurement_type)
                    {
                        numof_free_slots++;
                        return;
                    }

                    if(measurement_type.measurement_type != measurement.measurement_metadata.measurement_type){
                        return;
                    }

                    auto entity_type = std::find_if(std::begin(measurement_types), std::end(measurement_types), [&](const Measurement_entity_type& type){
                        return type.measurement_type == measurement.measurement_metadata.measurement_type;
                    });
                    if(entity_type != std::end(measurement_types)){
                        measurement.print(base_timestamp, mcu_time_unix);
                    }
                });
            }

            ESP_LOGI("TM_MEAS","Num of free slots: %ld", numof_free_slots);
        };

        uint16_t mark_measurements_to_be_sent(time_t measurement_start_time, uint16_t measurement_period_sec, uint16_t maximum_measurements){

            /* Resort the array to be time based only, so that all measurements marked will be in the same time period */
            /* @TODO: what to do here? should older ready_to_send flagged items be cleared? or counted as this time period? or explicitly check offset?*/
            std::sort(std::begin(tm_measurement_list), std::end(tm_measurement_list),[](const Timestamped_measurement& meas1, const Timestamped_measurement& meas2) {
                return meas1.measurement_metadata.time_offset_secs < meas2.measurement_metadata.time_offset_secs;
            });

            if(measurement_start_time == 0){
                measurement_start_time = base_timestamp;
            }

            auto [offset_min, offset_max] = Timestamped_measurement_storage::get_minmax_base_offset_secs(measurement_start_time, measurement_period_sec);
            uint16_t numof_marked = 0;

            ESP_LOGI("","MARKING MEASUREMENTS BETWEEN OFFSETS: %ld and %ld to be sent",offset_min, offset_max);
            for(Timestamped_measurement& measurement : tm_measurement_list){

                if(numof_marked >= maximum_measurements){
                    break;
                }

                if((measurement.measurement_metadata.time_offset_secs >= offset_min) && (measurement.measurement_metadata.time_offset_secs <= offset_max))
                {
                    switch(measurement.measurement_metadata.log_status){
                        case Log_status_t::RDY_TO_SEND:
                            numof_marked++;
                            break;
                        case Log_status_t::UNPROCESSED:
                            measurement.measurement_metadata.log_status = Log_status_t::RDY_TO_SEND;
                            numof_marked++;
                            break;
                        case Log_status_t::SEND_FAIL:
                            measurement.measurement_metadata.log_status = Log_status_t::RDY_TO_SEND;
                            numof_marked++;
                            break;
                        default:
                            continue;
                    }   
                }
                else if(measurement.measurement_metadata.log_status == Log_status_t::RDY_TO_SEND)
                {
                    measurement.measurement_metadata.log_status = Log_status_t::UNPROCESSED;
                }
            }
            //ESP_LOGI("","Requested a max of %ld measurements in the period of %ld and %ld to be sent, marked %ld/%ld to be sent.",offset_min, offset_max);
            return numof_marked;
        };

        /* NOTE: intentionally avoided templated fns or passing size of buffer as arg to discouridge use of this function with variable sizes.
         Pass the buffer meant for 'service data' payload directly, this will copy chars and done. No using this fn for other random stuff pls. */
        /* This is for web transfer as part of JSON object, @TODO: make something similar for logging to SD card. */
        esp_err_t get_servicedata_from_marked_measurements() /* char (&transport_buffer)[20]*/
        {
            constexpr uint16_t values_str_size{50};
            char value_str_buffer[values_str_size] = {};

            constexpr uint16_t service_data_str_size{values_str_size*5};
            char service_data_str[service_data_str_size] = {};

            uint8_t in_buffer = 0;
            uint8_t remaining = 0;

            for(Timestamped_measurement& measurement : tm_measurement_list){

                /* clear value string buffer */
                std::fill(value_str_buffer, (value_str_buffer + values_str_size), '\0');

                if(measurement.measurement_metadata.log_status == Log_status_t::RDY_TO_SEND ||
                    measurement.measurement_metadata.log_status == Log_status_t::IN_PROGRESS ||
                    measurement.measurement_metadata.log_status == Log_status_t::SEND_RETRY){

                    if(strlen(service_data_str) < (service_data_str_size - values_str_size)){
                        /* Add new service data */
                        measurement.get_service_data_keyval(value_str_buffer);
                        ESP_LOGI("KEYVALUE:","%s",value_str_buffer);
                        strncat(service_data_str, value_str_buffer, 50);
                        in_buffer++;
                        measurement.measurement_metadata.log_status = Log_status_t::IN_PROGRESS;
                    }
                    else{
                        remaining++;
                        break;
                    }
                }
            }

            ESP_LOGI("SERVICE_DATA:","%d ready to send, %d remaining.",in_buffer, remaining);
            ESP_LOGI("SERVICE_DATA:","%s",service_data_str);

            return ESP_OK;
        }
};

