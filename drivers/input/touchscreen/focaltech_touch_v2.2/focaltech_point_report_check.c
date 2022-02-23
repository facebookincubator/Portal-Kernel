#include "focaltech_core.h"

#if FTS_POINT_REPORT_CHECK_EN
#define POINT_REPORT_CHECK_WAIT_TIME			200

static void fts_prc_func(struct work_struct *work)
{
	struct fts_ts_data *ts_data = container_of(work, struct fts_ts_data,
						   prc_work.work);
	struct input_dev *input_dev = ts_data->input_dev;

#if FTS_MT_PROTOCOL_B_EN
	u32 finger_count = 0;
#endif

	FTS_FUNC_ENTER();
	mutex_lock(&ts_data->report_mutex);

#if FTS_MT_PROTOCOL_B_EN
	for (finger_count = 0;
	     finger_count < ts_data->pdata->max_touch_number; finger_count++) {
		input_mt_slot(input_dev, finger_count);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
	}
#else
	input_mt_sync(input_dev);
#endif
	input_report_key(input_dev, BTN_TOUCH, 0);
	input_sync(input_dev);

	mutex_unlock(&ts_data->report_mutex);

	FTS_FUNC_EXIT();
}
void fts_prc_queue_work(struct fts_ts_data *ts_data)
{
	cancel_delayed_work(&ts_data->prc_work);
	queue_delayed_work(ts_data->ts_workqueue, &ts_data->prc_work,
	msecs_to_jiffies(POINT_REPORT_CHECK_WAIT_TIME));
}

int fts_point_report_check_init(struct fts_ts_data *ts_data)
{
	FTS_FUNC_ENTER();

	if (ts_data->ts_workqueue) {
		INIT_DELAYED_WORK(&ts_data->prc_work, fts_prc_func);
	} else {
		FTS_ERROR("fts workqueue is NULL, can't run point report check function");
		return -EINVAL;
	}

	FTS_FUNC_EXIT();
	return 0;
}

int fts_point_report_check_exit(struct fts_ts_data *ts_data)
{
	FTS_FUNC_ENTER();

	FTS_FUNC_EXIT();
	return 0;
}
#endif /* FTS_POINT_REPORT_CHECK_EN */
