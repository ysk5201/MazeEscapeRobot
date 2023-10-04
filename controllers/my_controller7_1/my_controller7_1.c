/*
 * File:          my_controller7_1.c
 * Date:
 * Description:   最終課題
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 * robot.h  :ロボットノード
 * motor.h  :モータモード
 * camera.h  :カメラモード
 * led.h     :LEDノード
 * distance_sensor.h  :距離センサノード
 */
#include <stdio.h>
#include <string.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/camera.h>
#include <webots/led.h>
#include <webots/distance_sensor.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 256            //ロボットの制御周期[ms]
#define LEFT 0                   //左右を識別するための定数(左を0とする)
#define RIGHT 1                  //左右を識別するための定数(右を1とする)
  #define MAX_SPEED 6.28         //左・右車輪の指令角速度の最大値[rad/s]
#define CAMERA_TIME_STEP 1024    //カメラ画像の更新周期[ms]
#define NUM_LEDS 10              //LEDの数
#define NUM_DIST_SENS 8          //距離センサの数

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{
  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
  double speed[2]; //左・右車輪の指令角速度[rad/s]
  double current_time; //現在時刻を格納する
  double current_time_from_see_green=0; //緑の壁を見たときの時刻
  double current_time_from_see_red=0; //赤の壁を見たときの時刻
  int i;
  bool led_value[NUM_LEDS] = {false};           //各LEDの点灯消灯の指令値(初期化は消灯に設定)
  double dist_value[NUM_DIST_SENS] = {0.0};     //距離センサの測定値
  double sum_dist_value[NUM_DIST_SENS] = {0.0}; //距離センサ測定値の合計
  double avr_dist_value[NUM_DIST_SENS] = {0.0}; //距離センサの定常偏差(平均値)
  double nrm_dist_value[NUM_DIST_SENS] = {0.0}; //距離センサの測定値(校正後)
  int sampling_count = 0;     //キャリブレーションのためのサンプリング回数
  char device_name[13];       //デバイス名を格納する文字列
  const unsigned char *image; //撮影画像のピクセル色情報を格納するメモリのポインタ
  int width=0, height=0;      //画像の幅と高さを格納する変数
  int pixel_x=0, pixel_y=0;   //ピクセルのx座標とy座標
  int r=0, g=0, b=0;          //ピクセルの色情報
  int count_blue_pixel=0;     //青のピクセルの数
  int count_green_pixel=0;    //緑のピクセルの数
  int count_red_pixel=0;      //赤のピクセルの数
  
  WbDeviceTag motor[2];                        //左・右車輪のモータ
  WbDeviceTag camera;                          //カメラ
  WbDeviceTag led[NUM_LEDS];                   //LED (NUM_LEDS 個 = 10個)
  WbDeviceTag distance_sensors[NUM_DIST_SENS]; //距離センサ
  
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  motor[LEFT] = wb_robot_get_device("left wheel motor");   //左モータのタグの取得
  motor[RIGHT] = wb_robot_get_device("right wheel motor"); //右モータのタグの取得
  wb_motor_set_position(motor[LEFT], INFINITY);  //各速度指令モードに設定(左)
  wb_motor_set_position(motor[RIGHT], INFINITY); //各速度指令モードに設定(右)
  wb_motor_set_velocity(motor[LEFT], 0.0);  //左モータを停止させる
  wb_motor_set_velocity(motor[RIGHT], 0.0); //右モータを停止させる
  printf("Initialize motors... done\n");
  
  camera = wb_robot_get_device("camera");     //カメラのタグの取得
  wb_camera_enable(camera, CAMERA_TIME_STEP); //CAMERA_TIME_STEP周期で動作開始
  width = wb_camera_get_width(camera);        //カメラ画像の幅を取得
  height = wb_camera_get_height(camera);      //カメラ画像の高さを取得
  printf("Initialize camera ... done\n");
  
  strcpy(device_name, "led0");                 //デバイス名の設定
  for(i=0; i<NUM_LEDS; i++) {                  //全てのLEDについて繰り返す
    device_name[3] = '0' + i;                  //先頭から4文字目(数字)の文字コードを設定
    led[i] = wb_robot_get_device(device_name); //i番目のLEDのタグの取得
    wb_led_set(led[i], led_value[i]);          //LEDを初期状態にする
  }
  printf("Initialize LEDs ... done\n");
  
  strcpy(device_name, "ps0");                                  //デバイス名の設定
    for(i=0; i<NUM_DIST_SENS; i++) {                           //全ての距離センサについて繰り返す
    device_name[2] = '0' + i;                                  //先頭から3文字目(数字)の文字コードを設定
    distance_sensors[i] = wb_robot_get_device(device_name);    //i番目の距離センサのタグの取得
    wb_distance_sensor_enable(distance_sensors[i], TIME_STEP); //TIME_STEP周期で動作開始
  }
  printf("Initialize distance sensors ... done\n");
  
  //フラグの生成
  int color_num = 0; //blue=1, green=2, red=3
  
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {

    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
     current_time = wb_robot_get_time();  //現在時刻を取得する
     image = wb_camera_get_image(camera); //画像情報のポインタを取得

    /* Process sensor data here */
    for(i=0; i<NUM_DIST_SENS; i++) {      //全ての距離センサの測定値を取得
      dist_value[i] = wb_distance_sensor_get_value(distance_sensors[i]);
      
      if(wb_robot_get_mode() != WB_MODE_SIMULATION)
        dist_value[i] = (double)(unsigned short)dist_value[i];
    }
    count_blue_pixel=0;
    count_green_pixel=0;
    count_red_pixel=0;
     
     for(pixel_x = 0; pixel_x < width; pixel_x++){
         for(pixel_y = 0; pixel_y < height; pixel_y++){
             // r = 赤成分, g = 緑成分, b = 青成分, いずれも0~255を返す
             r = wb_camera_image_get_red(image, width, pixel_x, pixel_y);
             g = wb_camera_image_get_green(image, width, pixel_x, pixel_y);
             b = wb_camera_image_get_blue(image, width, pixel_x, pixel_y);
             //bが100より大きく、r,gが80より小さいピクセルを青とする
             if(r < 80 && g < 80 && b > 100){
                 count_blue_pixel++;
             }
             //gが100より大きく、r,bが80より小さいピクセルを青とする
             if(r < 80 && g > 100 && b < 80){
                 count_green_pixel++;
             }
             //rが100より大きく、b,gが80より小さいピクセルを青とする
             if(r > 100 && g < 80 && b < 80){
                 count_red_pixel++;
             }
         }
     }
     
     // 見た色によってcolor_numを変える
     
     //青色見たとき
     if (count_blue_pixel > 800) {
       color_num = 1;
     }
     //緑色見たとき
     if (count_green_pixel > 1000) {
       color_num = 2;
       current_time_from_see_green = wb_robot_get_time();
     }
     //赤色見たとき
     if (count_red_pixel > 1800 && !(color_num == 3)) { //一度カメラが赤色を認識したらそれ以上は更新しない
       color_num = 3;
       current_time_from_see_red = wb_robot_get_time();
     }
     
    
    
    /* 情報処理 ・ 制御計算 */
    
    //初めの10秒間ロボット後部のLEDをつけた状態で静止
    if(current_time < 10.0){      //シミュレーション開始から10秒以内なら
      if(current_time > 1.0){
        led_value[4] = true;      //後部のLEDを点灯
        sampling_count++;         //サンプリング回数を記録
        for(i=0; i<NUM_DIST_SENS; i++) sum_dist_value[i] += dist_value[i]; //測定値の総和を算出
      }
      for(i=0; i<NUM_DIST_SENS; i++)
        nrm_dist_value[i] = dist_value[i]; //キャリブレーション中は実測値を表示

      speed[LEFT] = 0.0;
      speed[RIGHT] = 0.0;
    }
    //青色の壁を見たとき
    //右手法で走行
    else if(color_num == 1){
        led_value[4] = false;
      for(i=0; i<NUM_DIST_SENS; i++){
        avr_dist_value[i] = sum_dist_value[i] / sampling_count;
        nrm_dist_value[i] = dist_value[i] - avr_dist_value[i];
      }
      
      if(nrm_dist_value[0] > 100 || nrm_dist_value[7] > 100){ //正面に壁があったら
        speed[LEFT] = -0.2 * MAX_SPEED;
        speed[RIGHT] = 0.2 * MAX_SPEED;
      }
      else if(nrm_dist_value[2] > 100){ //右に壁があったら
        speed[LEFT] = 0.1 * MAX_SPEED;
        speed[RIGHT] = 0.16 * MAX_SPEED;
      }
      else{
        speed[LEFT] = 0.27 * MAX_SPEED;
        speed[RIGHT] = 0.12 * MAX_SPEED;
      }
    }
    //緑色の壁を見たとき
    else if(color_num == 2){
        led_value[4] = false;
      for(i=0; i<NUM_DIST_SENS; i++){
        avr_dist_value[i] = sum_dist_value[i] / sampling_count;
        nrm_dist_value[i] = dist_value[i] - avr_dist_value[i];
      }
      if((current_time - current_time_from_see_green) < 8.0){ //カメラが緑色を認識してから8.0秒まで
        speed[LEFT] = 1.0;
        speed[RIGHT] = 1.8;
        printf("左旋回\n");
      }
      else if((current_time - current_time_from_see_green) < 18.0){ //カメラが緑色を認識してから18.0秒まで
        speed[LEFT] = 1.9;
        speed[RIGHT] = 1.1;
        printf("右旋回\n");
      }
      else{                        //カメラが緑色を認識してから18.0以降
        speed[LEFT] = 1.0;
        speed[RIGHT] = 1.8;
        printf("左旋回\n");
      }
    }
    //赤色の壁を見たとき
    else if(color_num == 3){
      if((current_time - current_time_from_see_red) < 10.0){ //カメラが赤色を認識してから10.0秒まで
        for(i=0; i<NUM_LEDS; i++) {
          led_value[i] = true;
        }
        speed[LEFT] = 0.0 * MAX_SPEED;
        speed[RIGHT] = 0.0 * MAX_SPEED;
        printf("全てのLEDを点灯\n");
      }
      else if((current_time - current_time_from_see_red) < 20.0){ //カメラが赤色を認識してから20.0秒まで
        for(i=0; i<NUM_LEDS; i++) {
          if((int)(current_time - current_time_from_see_red) % 2 == 0){
            led_value[i] = true;
          }
          else{
            led_value[i] = false;
          }
        }
        speed[LEFT] = 0.0;
        speed[RIGHT] = 0.0;
        printf("全てのLEDを1秒おきに点灯\n");
      }
      else if((current_time - current_time_from_see_red) < 31.0){ //カメラが赤色を認識してから31.0秒まで
        double timing;
    
        timing = (int)(current_time/10);
        timing = current_time - timing*10;
        if ( timing >= 1.0 && timing <(double)NUM_LEDS + 1.0)
                   led_value[(int)(timing) -1] = true;
        if ( timing >= 0.0 && timing <=1.0 ){
                   for (i=0; i < NUM_LEDS; i++){
                            led_value[i] = false;
                   }
        }
        speed[LEFT] = 0.0;
        speed[RIGHT] = 0.0;
        printf("複数のLEDが1秒おきに順番に点灯\n");
      }
      else{                                    //カメラが赤色を認識してから31.0秒以降
        for(i=0; i<NUM_LEDS; i++) {
          led_value[i] = false;
        }
        speed[LEFT] = 0.0;
        speed[RIGHT] = 0.0;
        printf("LEDを消灯\n");
      }
    }
    //左手法で走行
    else{
      led_value[4] = false;
      for(i=0; i<NUM_DIST_SENS; i++){
        avr_dist_value[i] = sum_dist_value[i] / sampling_count;
        nrm_dist_value[i] = dist_value[i] - avr_dist_value[i];
      }
      
      if(nrm_dist_value[0] > 100 || nrm_dist_value[7] > 100){ //正面に壁があったら
        speed[LEFT] = 0.2 * MAX_SPEED;
        speed[RIGHT] = -0.2 * MAX_SPEED;
      }
      else if(nrm_dist_value[5] > 100){ //右に壁があったら
        speed[LEFT] = 0.15 * MAX_SPEED;
        speed[RIGHT] = 0.1 * MAX_SPEED;
      }
      else{
        speed[LEFT] = 0.1 * MAX_SPEED;
        speed[RIGHT] = 0.25 * MAX_SPEED;
      }
    }
    
      

    /*
     * Enter here functions to send actuator commands, like:
     * wb_differential_wheels_set_speed(100.0,100.0);
     */
     for(i=0; i<NUM_LEDS; i++) {
      wb_led_set(led[i], led_value[i]);
    }
     
     wb_motor_set_velocity(motor[LEFT], speed[LEFT]);
     wb_motor_set_velocity(motor[RIGHT], speed[RIGHT]);
     printf("TIME: %.3lf,  SPEED: %.3lf, %.3lf\n", current_time, speed[LEFT], speed[RIGHT]);
     printf("TIME_SEE_GREEN: %.3lf\n", current_time_from_see_green); //緑の壁を見た時刻
     printf("TIME_SEE_RED: %.3lf\n", current_time_from_see_red);     //赤の壁を見た時刻
     printf("DIST. SENSOR: %5.0lf, %5.0lf, %5.0lf, %5.0lf, %5.0lf, %5.0lf, %5.0lf, %5.0lf\n",
       nrm_dist_value[0], nrm_dist_value[1], nrm_dist_value[2], nrm_dist_value[3],
       nrm_dist_value[4], nrm_dist_value[5], nrm_dist_value[6], nrm_dist_value[7]);
     printf("BluePixel: %d\n", count_blue_pixel);    //青ピクセルの値を表示
     printf("GreenPixel: %d\n", count_green_pixel);  //緑ピクセルの値を表示
     printf("RedPixel: %d\n", count_red_pixel);      //赤ピクセルの値を表示
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
