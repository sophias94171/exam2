#include "mbed.h"

#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"
////////
#include "mbed_rpc.h"

#include "uLCD_4DGL.h"

////////
#include "accelerometer_handler.h"

#include "config.h"

#include "magic_wand_model_data.h"

#include "tensorflow/lite/c/common.h"

#include "tensorflow/lite/micro/kernels/micro_ops.h"

#include "tensorflow/lite/micro/micro_error_reporter.h"

#include "tensorflow/lite/micro/micro_interpreter.h"

#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"

#include "tensorflow/lite/schema/schema_generated.h"

#include "tensorflow/lite/version.h"

#include "stm32l475e_iot01_accelero.h"

#include <math.h>
//////
uLCD_4DGL uLCD(D1, D0, D2);

int index_number;

Thread thread(osPriorityHigh);
Thread t;
EventQueue queue(32 * EVENTS_EVENT_SIZE);

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
//DigitalIn mypin(USER_BUTTON);
//Timeout flipper;//flipper.attach(&flip, 2s);
///////////////////////////////////
void GetACC(Arguments *in, Reply *out);
void gestureUIMode(Arguments *in, Reply *out);


RPCFunction rpcGetACC(&GetACC, "GetACC");
RPCFunction gestureUIMode(&gestureUIMode, "gestureUIMode");
BufferedSerial pc(USBTX, USBRX);
///////////////////////////////
// GLOBAL VARIABLES
WiFiInterface *wifi;
MQTT::Client<MQTTNetwork, Countdown> *client_ptr;
MQTTNetwork *mqttNetwork;

InterruptIn btnRecord(USER_BUTTON);
int idR[32] = {0};
int indexR = 0;
//InterruptIn btn3(SW3);
volatile int message_num = 0;
volatile int arrivedcount = 0;
volatile bool closed = false;
constexpr int kTensorArenaSize = 60 * 1024;
uint8_t tensor_arena[kTensorArenaSize];
const char* topic = "Mbed";



void messageArrived(MQTT::MessageData& md) {
    MQTT::Message &message = md.message;
    char msg[300];
    sprintf(msg, "Message arrived: QoS%d, retained %d, dup %d, packetID %d\r\n", message.qos, message.retained, message.dup, message.id);
    printf(msg);
    ThisThread::sleep_for(1000ms);
    char payload[300];
    sprintf(payload, "Payload %.*s\r\n", message.payloadlen, (char*)message.payload);
    printf(payload);
    ++arrivedcount;
}

void publish_message_1(float angle)
{
    message_num++;
    MQTT::Message message;
    char buff[100];
    sprintf(buff, "selected angle: %f \n", angle);
    message.qos = MQTT::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = (void*) buff;
    message.payloadlen = strlen(buff) + 1;
    int rc = client_ptr->publish(topic, message);
}

void publish_message_2(int i, float ang){
    message_num++;
    MQTT::Message message;
    char buff[100];
    sprintf(buff, " tilt event #%d: %f \n", i, ang);
    message.qos = MQTT::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = (void*) buff;
    message.payloadlen = strlen(buff) + 1;
    int rc = client_ptr->publish(topic, message);
    printf("rc:  %d\r\n", rc);
    printf("Puslish message: %s\r\n", buff);
}

void close_mqtt() {
    closed = true;
}


int PredictGesture(float *output)
{

    // How many times the most recent gesture has been matched in a row

    static int continuous_count = 0;

    // The result of the last prediction

    static int last_predict = -1;

    // Find whichever output has a probability > 0.8 (they sum to 1)

    int this_predict = -1;

    for (int i = 0; i < label_num; i++)
    {

        if (output[i] > 0.8)
            this_predict = i;
    }

    // No gesture was detected above the threshold

    if (this_predict == -1)
    {

        continuous_count = 0;

        last_predict = label_num;

        return label_num;
    }

    if (last_predict == this_predict)
    {

        continuous_count += 1;
    }
    else
    {

        continuous_count = 0;
    }

    last_predict = this_predict;

    // If we haven't yet had enough consecutive matches for this gesture,

    // report a negative result

    if (continuous_count < config.consecutiveInferenceThresholds[this_predict])
    {

        return label_num;
    }

    // Otherwise, we've seen a positive result, so clear all our variables

    // and report it

    continuous_count = 0;

    last_predict = -1;

    return this_predict;
}

int gesture(TfLiteTensor *model_input, tflite::ErrorReporter *error_reporter, tflite::MicroInterpreter *interpreter)
{
    float result = 0;
    bool should_clear_buffer = false;
    bool got_data = false;
    int input_length = model_input->bytes / sizeof(float);
    int record_flag = 0;
    int gesture_index;
    while (1)
    {
        // Attempt to read new data from the accelerometer

        got_data = ReadAccelerometer(error_reporter, model_input->data.f,

                                     input_length, should_clear_buffer);

        // If there was no new data,

        // don't try to clear the buffer again and wait until next time

        if (!got_data)
        {
            should_clear_buffer = false;
            continue;
        }

        // Run inference, and report any error
        TfLiteStatus invoke_status = interpreter->Invoke();
        if (invoke_status != kTfLiteOk)
        {
            error_reporter->Report("Invoke failed on index: %d\n", begin_index);
            continue;
        }

        // Analyze the results to obtain a prediction
        gesture_index = PredictGesture(interpreter->output(0)->data.f);
        // Clear the buffer next time we read data
        should_clear_buffer = gesture_index < label_num;
        if (gesture_index < label_num) error_reporter->Report(config.output_message[gesture_index]);
    

        ThisThread::sleep_for(100ms);
    }
    return gesture_index;
}
void gestureUIMode(Arguments *in, Reply *out)
{
    led3 = 1;
    char buffer[200];


    static tflite::MicroErrorReporter micro_error_reporter;

    tflite::ErrorReporter *error_reporter = &micro_error_reporter;


    const tflite::Model *model = tflite::GetModel(g_magic_wand_model_data);

    if (model->version() != TFLITE_SCHEMA_VERSION)
    {

        error_reporter->Report(

            "Model provided is schema version %d not equal "

            "to supported version %d.",

            model->version(), TFLITE_SCHEMA_VERSION);

        return ;
    }

    // Pull in only the operation implementations we need.

    // This relies on a complete list of all the ops needed by this graph.

    // An easier approach is to just use the AllOpsResolver, but this will

    // incur some penalty in code space for op implementations that are not

    // needed by this graph.

    static tflite::MicroOpResolver<6> micro_op_resolver;

    micro_op_resolver.AddBuiltin(

        tflite::BuiltinOperator_DEPTHWISE_CONV_2D,

        tflite::ops::micro::Register_DEPTHWISE_CONV_2D());

    micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_MAX_POOL_2D,

                                 tflite::ops::micro::Register_MAX_POOL_2D());

    micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_CONV_2D,

                                 tflite::ops::micro::Register_CONV_2D());

    micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_FULLY_CONNECTED,

                                 tflite::ops::micro::Register_FULLY_CONNECTED());

    micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_SOFTMAX,

                                 tflite::ops::micro::Register_SOFTMAX());

    micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_RESHAPE,

                                 tflite::ops::micro::Register_RESHAPE(), 1);

    // Build an interpreter to run the model with

    static tflite::MicroInterpreter static_interpreter(

        model, micro_op_resolver, tensor_arena, kTensorArenaSize, error_reporter);

    tflite::MicroInterpreter *interpreter = &static_interpreter;

    // Allocate memory from the tensor_arena for the model's tensors

    interpreter->AllocateTensors();

    // Obtain pointer to the model's input tensor

    TfLiteTensor *model_input = interpreter->input(0);

    if ((model_input->dims->size != 4) || (model_input->dims->data[0] != 1) ||

        (model_input->dims->data[1] != config.seq_length) ||

        (model_input->dims->data[2] != kChannelNumber) ||

        (model_input->type != kTfLiteFloat32))
    {

        error_reporter->Report("Bad input tensor parameters in model");

        return ;
    }

    TfLiteStatus setup_status = SetupAccelerometer(error_reporter);

    if (setup_status != kTfLiteOk)
    {

        error_reporter->Report("Set up failed\n");

        return ;
    }

    error_reporter->Report("Set up successful...\n");

    int index_number = gesture(model_input, error_reporter, interpreter);
    
    return;

}


int setup_wifi(){
    wifi = WiFiInterface::get_default_instance();
    //printf("1\n");

    if (!wifi) {
            printf("ERROR: No WiFiInterface found.\r\n");
            return -1;
    }


    printf("\nConnecting to %s...\r\n", MBED_CONF_APP_WIFI_SSID);
    int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
    if (ret != 0) {
            printf("\nConnection error: %d\r\n", ret);
            return -1;
    }
    //printf("2\n");
    NetworkInterface* net = wifi;
    mqttNetwork = new MQTTNetwork(net);
    client_ptr = new MQTT::Client<MQTTNetwork, Countdown>(*mqttNetwork);
    //TODO: revise host to your IP
    const char* host = "172.20.10.2";
    printf("Connecting to TCP network...\r\n");
    //printf("3\n");
    SocketAddress sockAddr;
    sockAddr.set_ip_address(host);
    sockAddr.set_port(1883);
    //printf("4\n");
    printf("address is %s/%d\r\n", (sockAddr.get_ip_address() ? sockAddr.get_ip_address() : "None"),  (sockAddr.get_port() ? sockAddr.get_port() : 0) ); //check setting
    int rc = mqttNetwork->connect(sockAddr);//(host, 1883);
    if (rc != 0) {
            printf("Connection error.");
            return -1;
    }
    printf("Successfully connected!\r\n");
    //printf("5\n");
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = "Mbed";

    if ((rc = client_ptr->connect(data)) != 0){
            printf("Fail to connect MQTT\r\n");
    }
    if (client_ptr->subscribe(topic, MQTT::QOS0, messageArrived) != 0){
            printf("Fail to subscribe\r\n");
    }
    //printf("6\n");
    //client_ptr = &client;
    //printf("7\n");
    return 0;
}

void record(void) {

   int16_t pDataXYZ[3] = {0};
   BSP_ACCELERO_AccGetXYZ(pDataXYZ);

   printf("%d, %d, %d\n", pDataXYZ[0], pDataXYZ[1], pDataXYZ[2]);

}


void startRecord(void) {

   printf("---start---\n");

   idR[indexR++] = queue.call_every(1ms, record);

   indexR = indexR % 32;

}


void stopRecord(void) {

   printf("---stop---\n");

   for (auto &i : idR)

      queue.cancel(i);

}

void GetACC(Arguments *in, Reply *out)
{
    char buffer[200];
   printf("Start accelerometer init\n");

   BSP_ACCELERO_Init();

   t.start(callback(&queue, &EventQueue::dispatch_forever));

   btnRecord.fall(queue.event(startRecord));

   btnRecord.rise(queue.event(stopRecord));

}


int main(void)
{
 
    uLCD.locate(0, 0);
    uLCD.text_width(2); //4X size text
    uLCD.text_height(2);
    thread.start(callback(&queue, &EventQueue::dispatch_forever));
    //printf("!!!!!!!!!!!!!!!!!!!!!!!\n");
    setup_wifi();
    //printf("!!!!!!!!!!!!!!!!!!!!!!!\n");

    //////////////////////////////////////
    char buf[256], outbuf[256];

    FILE *devin = fdopen(&pc, "r");
    FILE *devout = fdopen(&pc, "w");
    while(1) {
        memset(buf, 0, 256);
        for (int i = 0; ; i++) {
            char recv = fgetc(devin);
            if (recv == '\n') {
                printf("\r\n");
                break;
            }
            buf[i] = fputc(recv, devout);
        }
        //Call the static call method on the RPC class
        RPC::call(buf, outbuf);
        printf("%s\r\n", outbuf);
    }
    ///////////////////////////////////////
    
 
}