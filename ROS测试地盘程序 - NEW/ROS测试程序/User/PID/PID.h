
struct PID
	{
	float Kp;
	float Ki;
	float Kd;
	float error_0;//��������
	float error_1;//һ��г������
	float error_2;//����г������
	long  Sum_error;
	float OutputValue;//ʵ�������
	float OwenValue;//�����ʱ�ı�׼�����
	};
float PID_calculate( struct PID *Control,float CurrentValue);      //λ��PID����
	
	struct PID_R
	{
	float rKp;
	float rKi;
	float rKd;
	float rerror_0;//��������
	float rerror_1;//һ��г������
	float rerror_2;//����г������
	long  rSum_error;
	float rOutputValue;//ʵ�������
	float rOwenValue;//�����ʱ�ı�׼�����
	};
float PID_calculate_R( struct PID_R *Control_R,float CurrentValue);      //λ��PID����
