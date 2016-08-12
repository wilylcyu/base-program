
struct PID
	{
	float Kp;
	float Ki;
	float Kd;
	float error_0;//基波分量
	float error_1;//一次谐波分量
	float error_2;//二次谐波分量
	long  Sum_error;
	float OutputValue;//实际输出量
	float OwenValue;//零误差时的标准输出量
	};
float PID_calculate( struct PID *Control,float CurrentValue);      //位置PID计算
	
	struct PID_R
	{
	float rKp;
	float rKi;
	float rKd;
	float rerror_0;//基波分量
	float rerror_1;//一次谐波分量
	float rerror_2;//二次谐波分量
	long  rSum_error;
	float rOutputValue;//实际输出量
	float rOwenValue;//零误差时的标准输出量
	};
float PID_calculate_R( struct PID_R *Control_R,float CurrentValue);      //位置PID计算
