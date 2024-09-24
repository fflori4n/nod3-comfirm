#include "cron_scheduler.h"


esp_err_t compute_next_exec_time(){
    return ESP_OK;
}

bool Cron_Scheduler::is_ready_to_execute(cron_job_t& job){

    /* TODO: check if RTC time is correct */

    /* TODO: write this crap*/
    if((cjob_status_t::cjob_unprocessed == job.job_status)){
         return true;
    }

    return false;
}