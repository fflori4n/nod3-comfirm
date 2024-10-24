#include <esp_err.h>
#include <esp_netif_sntp.h>

enum class job_status_t{
    job_slot_empty      = 0,
    job_unprocessed     = 1,
    job_not_executed    = 2,
    job_waiting         = 3,
    job_armed           = 4,
    job_in_progress     = 5,
    job_executed        = 6,
    job_error           = 7
};

/* NOTE: to use RTTI for the whole project uncomment the line CONFIG_COMPILER_CXX_RTTI in sdkconfig*/
typedef struct timed_job_t{

    job_status_t status;
    esp_err_t result{ESP_ERR_INVALID_RESPONSE};
    uint32_t timeout_seconds;
    time_t next_exec_unix_time;

    timed_job_t(void) {};
    timed_job_t(job_status_t status, uint32_t timeout_seconds, time_t next_exec_unix_time) : status(status), timeout_seconds(timeout_seconds), next_exec_unix_time(next_exec_unix_time) {};
    
    virtual void print() const {
        ESP_LOGI("OBJ timed_job_t", "status: %d, res: %d, timeout:%ld, next exec:%lld", (uint8_t)status, (uint8_t)result, timeout_seconds, next_exec_unix_time);
    }
};

typedef struct cron_job_t : timed_job_t{

    char* cron_str = "\0";

    cron_job_t(void) {};
    cron_job_t(job_status_t status, uint32_t timeout_seconds, char* cron_str) : timed_job_t(status,timeout_seconds,0), cron_str(cron_str) {};
    
    void print() const override{
        ESP_LOGI("OBJ cron_job_t", "status: %d, res: %d, timeout:%ld, next exec:%lld, cron: %s", (uint8_t)status, (uint8_t)result, timeout_seconds, next_exec_unix_time, cron_str);
    }
};

class Cron_Scheduler{

    private:

    static esp_err_t compute_next_exec_time(cron_job_t& job);
    /*static uint16_t generate_job_hash(int8_t eventType, uint32_t startTime, uint32_t endTime);*/
    public:

    /* Will do separate lists for timed_job and for cron_job, cause I want to not deal with random pointer scopes, just store all obj. in this static list.*/
    static timed_job_t timed_job_queue[10];
    static cron_job_t cron_job_queue[10];

    static esp_err_t add_to_job_queue(timed_job_t new_job);

    static time_t arm_ms_before;

    virtual bool is_ready_to_execute(timed_job_t& job, time_t& unix_time);
    static void print_job(cron_job_t& job);

};