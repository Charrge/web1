#!/usr/bin/python
# -*- coding: utf-8 -*-

## 전제 조건 ########################################################################################################################################
# 컨베이어 속도 : 100mm/s
# 물체는 한 개씩 컨베이어 상에 공급 (공급주기는 Tact time 따라 달라질 수 있음) - 현재는 200mm


### 1. 모듈 호출 ####################################################################################################################################
from i611_MCS import *
from i611_extend import *
from i611_io import *
from i611shm import shm_read
from teachdata import *
import socket, time, math
### 로봇 객체 생성 및 초기화 #########################################################################################################################
rb = i611Robot()
_BASE = Base()
rb.open()
IOinit( rb )

#. 인터럽트 활성화 
rb.enable_interrupt(0,True) # 
rb.enable_interrupt(1,True) # 동작 중의 비상 정지 입력시의 예외 발생을 활성화
rb.enable_interrupt(2,True) # 
rb.enable_interrupt(3,True) # 

### 교시데이터 로드 및 좌표 생성 ######################################################################################################################
data = Teachdata("teach_data")

#. 팔레트 좌표 생성
pal0 = data.get_position("pos1",0)
pal0 = pal0.offset(dy=-76)
pal1 = data.get_position("pos1",1)
pal1 = pal1.offset(dy=-76)
pal2 = data.get_position("pos1",2)
pal2 = pal2.offset(dy=-76)
pal = Pallet()
pal.init_3( pal0, pal1, pal2,  4, 3 ) # k(0,0),i(1,0),j(0,1),행,열

#. 컨베이어벨트 상의 원점 지정
orig = data.get_position("pos1",3)

### 클라이언트 소켓 생성 및 서버 접속 ##################################################################################################################
sock_vision = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock_vision.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1)
sock_vision.connect(('192.168.0.7',49211)) # 서버 IP 주소 및 포트번호(agrument) 입력 - (Vision Cam 서버 IP주소, 포트)

### 글로벌 변수 초기화 ################################################################################################################################
time_recv=[0 for j in range(1000)]
first_captured=[0 for j in range(1000)]
capture_flag = [False for j in range(1000)]
PASS    = 1
FAIL    = 0
UNKNOWN = -1
DONE    = 2
passfail = [UNKNOWN for j in range(1000)]
pos = [[0]*3 for j in range(1000)]

RANGE = 160         # mm (WorkPlace 구분 Range)
ConveyorSPEED = 100 # mm/s
ConveyorLENGTH = 1450 # mm
timing_offset = 0.09
### 포토센서 인식 쓰레드 ##############################################################################################################################
def thread_photo():
    global time_recv
    i_p = 0
    while True:
        photo_input = int(shm_read(0x0100,1)) # 공유메모리에서 Digital Input 값을 읽어옴 // str -> 정수 변환
        # print photo_input & 0b100000000 == 0b100000000
        if photo_input & 0b100000000 == 0b100000000: # 포토센서 인식될 경우 IN9 == 1 (9번째 비트)
            time_recv[i_p] = time.time()         # 포토센서 인식된 타이밍을 저장
            i_p+=1                               # 인덱스 증가
            print i_p,"번째 포토센서 인식"
            time.sleep(0.5)                            # 한 물체에 대해 포토센서가 여러번 입력되는 오류(노이즈 등)를 방지
        time.sleep(0.001)                            # 쓰레드 반복 구동을 위한 딜레이
        
### 비전센서 인식 쓰레드 ##############################################################################################################################
def thread_vision():
    global pos, capture_flag, first_captured, passfail
    i_v=0
    while True:
        data_recv = sock_vision.recv(65535) # 비전캠으로부터 데이터 수신 [합불여부, X, Y, RZ]
        data_Target = data_recv.split(',')  
        if time.time()-time_recv[i_v] > 3 and data_Target[0] == '1.000000': # 포토센서인식에 해당하는 비전트리거 일치여부 및 합불여부 판단 (data_Target[0] == 합불여부 (1:Pass, 0:Fail))
            capture_flag[i_v] = True    # 물체 인식 플래그 1 : 전체 트래킹 동작에 사용
            first_captured[i_v] = True  # 물체 인식 플래그 2 : 인식된 위치로 이동하여 대기하는 동작에 사용
            passfail[i_v] = PASS        # 합불 여부 : True == 합격
            pos[i_v][0]=data_Target[2]  # 1 : 카메라에서 받은 y좌표 -> 로봇의 x좌표  
            pos[i_v][1]=data_Target[1]  # 0 : 카메라에서 받은 x좌표 -> 로봇의 y좌표
            pos[i_v][2]=data_Target[3]  # 2 : rz좌표 (아직 미사용)
            i_v+=1                      # 인덱스 증가
            print i_v,'번째 : 타겟 생성'
        else:
            capture_flag[i_v] = False   # 물체 인식 플래그 1 : 인식 실패
            first_captured[i_v] = False # 물체 인식 플래그 2 : 인식 실패
            passfail[i_v] = FAIL       # 합불 여부 : False == 불합격
            i_v+=1                      # 인덱스 증가
            print i_v,'번째 : 검출에러'
        time.sleep(0.001)             # 쓰레드 구동을 위한 타임딜레이
        
def main():
    global pos,capture_flag,first_captured # 글로벌 변수 선언

    ### 초기 설정 ####################################################################################################################################
    mp0 = MotionParam(jnt_speed = 10, lin_speed = 100, acctime = 0.4, dacctime = 0.4)               # 처음 코드 시작 후 초기 위치 이동 (천천히)
    mp1 = MotionParam(lin_speed = 3000, acctime = 0.4, dacctime = 0.4)                              # 카메라가 인식한 좌표 위치로 이동 후 대기
    mp2 = MotionParam(lin_speed = ConveyorSPEED, acctime = 0.2, dacctime = 0.2, overlap = 10)       # 컨베이어 속도와 일치시켜 이동
    mp3 = MotionParam(lin_speed = 2000, acctime = 0.3, dacctime = 0.3, overlap = 10)                # 물체 잡은 후 상승 모션
    mp4 = MotionParam(lin_speed=3000,acctime=0.25,dacctime=0.25,overlap=10)                         # 물체 피킹 후 팔레트 위치로 이동
    mp5 = MotionParam(lin_speed=1000,acctime=0.25,dacctime=0.25,overlap=10)                         # 팔레트 위치로 이동 후 내려놓고 올라오는 동작
    ### 쓰레드 구동 ###################################################################################################################################
    thcheck1 = threading.Thread(target=thread_photo)
    thcheck1.setDaemon(True)
    thcheck1.start()
    thcheck2 = threading.Thread(target=thread_vision)
    thcheck2.setDaemon(True)
    thcheck2.start()
    
    ### 초기화 동작 ###################################################################################################################################
    dout(24,'10')
    rb.motionparam(mp0)
    
    #. Z축 +50 만큼 회피 (초기위치 이동 시 경로 확보 목적)
    # p_list=shm_read(0x3000,6).split(',')
    # curpos = Position(1000.0*float(p_list[0]),1000.0*float(p_list[1]),1000.0*float(p_list[2]),
    #                 math.degrees(float(p_list[3])), math.degrees(float(p_list[4])), math.degrees(float(p_list[5])))
    # rb.move(curpos.offset(dz=25))

    #. 초기위치 이동
    p1=orig.offset(dy=-15,dz=-11)   # 컨베이어 벨트의 가운데에 위치하는지 확인하여 캘리브레이션 수행
    rb.move(p1)
    print math.sin(0.7*math.pi/180)*RANGE
    print math.tan(0.7*math.pi/180)*RANGE

    #. 인덱스 초기화
    i=0
    i_pal=0
    miss=[False for j in range(1000)]
    area=0
    ### 전체 동작 수행 ###################################################################################################################################
    try:
        rb.asyncm(1)
        while True:
            ## 1. 물체 검출 시 (Pass)
            if passfail[i] == PASS:
                # (포토 센서 인식 -> 현재)까지의 경과시간 저장
                now = time.time() - time_recv[i]     
                #. 인식된 좌표로 이동 후, 트래킹 시작 타이밍 동기화를 위해 대기
                if first_captured[i] == True:        
                    rb.motionparam(mp1)
                    print RANGE*math.tan(0.7*math.pi/180)*(1-area)
                    if area==0:
                        target_pos = p1.offset(dx=float(pos[i][0])+RANGE*(area-1),dy=float(pos[i][1]),dz=4,drz=float(pos[i][2]))
                    if area==1:
                        target_pos = p1.offset(dx=float(pos[i][0])+RANGE*(area-1),dy=float(pos[i][1]),drz=float(pos[i][2]))
                    if area==2:
                        target_pos = p1.offset(dx=float(pos[i][0])+RANGE*(area-1),dy=float(pos[i][1]),dz=-2,drz=float(pos[i][2]))
                    rb.line(target_pos)
                    first_captured[i] = False
                #. 트래킹 시작 타이밍 동기화
                if area==0:
                    timing_offset=0.04
                elif area==1:
                    timing_offset=0.09
                elif area==2:
                    timing_offset=0.12

                timing = float(ConveyorLENGTH)/float(ConveyorSPEED) + timing_offset - float(pos[i][0])/ConveyorSPEED + RANGE*0.01*(area-1)  # (컨베이어길이/속도)+ 시간보정치 - 포토센서인식시간보정치 + 피킹 area 시간
                if now < timing: # (포토센서-로봇 거리)/컨베이어속도 = 시간  [mm]/[mm/s]=[s]
                    time.sleep(0.001)
                    continue
                while True:  # PLC INTERLOCK 신호 확인
                    if int(shm_read(0x0100,1)) & 0b10 != 0b10:
                        break
                    print 'WAIT!! INTER LOCKED BY PLC'
                    rb.sleep(0.001)
                tact_time_ref = time.time()
                #. 컨베이어 속도와 동일하게 이동하며 물체를 Picking
                rb.motionparam(mp2)
                rb.line(target_pos.offset(dx=5,dz=-6.5)) 
                dout(24,'01')
                rb.line(target_pos.offset(dx=10,dz=-6.5))
                rb.motionparam(mp3)
                rb.line(target_pos.offset(dx=10,dz=12))
                
                #. 팔레타이징
                pos0 = pal.get_pos( (i_pal%12)%4, (i_pal%12)/4, 7 )     # 위치 정의
                pos1 = pal.get_pos( (i_pal%12)%4, (i_pal%12)/4, -15 )
                pos2 = pal.get_pos( (i_pal%12)%4, (i_pal%12)/4, 4 )    # 위치 정의
                rb.motionparam(mp4)
                rb.line(pos0)                                           # 팔레트 적재위치 상단으로 이동
                rb.motionparam(mp5)
                rb.line(pos1)                                           # 팔레트 적재위치로 이동
                rb.join()
                rb.asyncm(2)
                dout(24,'10')                                            # 공압 밸브 OFF
                rb.sleep(0.3)
                rb.asyncm(1)
                rb.line(pos2)
                rb.motionparam(mp4)     
                
                # #. 다음 타겟 피킹을 위한 area 설정
                # if timing+3*RANGE*0.01 <= time.time()-time_recv[i+1]: # -2
                #     area=0
                # elif timing+2*RANGE*0.01 <= time.time()-time_recv[i+1] and time.time()-time_recv[i+1] < timing+3*RANGE*0.01: # -1
                #     if area==0:
                #         area=0
                #     else:
                #         area-=1
                # elif timing+1*RANGE*0.01 <= time.time()-time_recv[i+1] and time.time()-time_recv[i+1] < timing+2*RANGE*0.01: # +0
                #     print '그대로'
                # elif timing+0*RANGE*0.01 <= time.time()-time_recv[i+1] and time.time()-time_recv[i+1] < timing+1*RANGE*0.01: # +1
                #     if area==2:
                #         print '물체를 잡을 수 없습니다.. 통과시킵니다.'
                #         i+=1
                #     else:
                #         area+=1

                while True:
                    if 14.59 - float(pos[i+1][0])/ConveyorSPEED + RANGE/100 <= time.time()-time_recv[i+1]:
                        if passfail[i+2]==True:
                            print '물체를 잡을 수 없습니다.. 흘려보냅니다.'
                            i+=1
                    elif 14.59 - float(pos[i+1][0])/ConveyorSPEED <= time.time()-time_recv[i+1] and time.time()-time_recv[i+1] < 14.59 - float(pos[i+1][0])/ConveyorSPEED + RANGE/100:
                        area=2
                        break
                    elif 14.59 - float(pos[i+1][0])/ConveyorSPEED - RANGE/100 <= time.time()-time_recv[i+1] and time.time()-time_recv[i+1] < 14.59 - float(pos[i+1][0])/ConveyorSPEED:
                        area=1
                        break
                    elif time.time()-time_recv[i+1] < 14.59 - float(pos[i+1][0])/ConveyorSPEED - RANGE/100 :
                        area=0
                        break
                    
                #. 대기 위치 이동
                if passfail[i+1]==PASS: # 다음 타겟의 위치가 확보되었을 경우 : 다음 타겟 좌표로 이동 후 대기
                    rb.line(p1.offset(dx=float(pos[i+1][0]),dy=float(pos[i+1][1]),drz=float(pos[i+1][2])))
                else:                   # 다음 타겟의 위치가 아직 확보되지 않은 경우 : 초기 위치로 이동 후 대기
                    rb.line(p1)
                rb.join()
                rb.asyncm(2)
                tact_time = time.time()-tact_time_ref
                print 'tact_time : ', tact_time
                print 'time.time()-time_recv[i+1]', time.time()-time_recv[i+1]
                print 'timing :' ,timing


                # #. 다음 타겟 피킹을 위한 area 설정
                # while True:
                #     if timing+3*tact_time <= time.time()-time_recv[i+1]: # -2
                #         area=0
                #         print 'condition 1'
                #         break
                #     elif timing+2*tact_time <= time.time()-time_recv[i+1] and time.time()-time_recv[i+1] < timing+3*tact_time: # -1
                #         area=0
                #         print 'condition 2'
                #         break
                #     elif timing+1*tact_time <= time.time()-time_recv[i+1] and time.time()-time_recv[i+1] < timing+2*tact_time: # +0
                #         area=1
                #         print 'condition 3'
                #         break
                #     elif timing+0*tact_time <= time.time()-time_recv[i+1] and time.time()-time_recv[i+1] < timing+1*tact_time: # +1
                #         area=2
                #         print 'condition 4'
                #         break
                #     elif timing+0*tact_time > time.time()-time_recv[i+1]:
                #         i+=1
                #         print 'condition 5'
                #         '잡을 수 없는 물체입니다.'
                #         continue



                rb.asyncm(1)
                #. PLC로 팔레트 작업 완료 신호 전송
                if i_pal%12==11:
                    rb.asyncm(2)
                    print 'dout 보냄'
                    dout(17, '1')
                    rb.sleep(10)    # 더 줄이기
                    dout(17, '0')
                    rb.asyncm(1)
                passfail[i]=DONE
                i+=1
                i_pal+=1
                 
            ## 2. 물체 검출 실패시 (Fail)
            elif passfail[i]==FAIL:
                i+=1
            ## 3. IDLE STATE (passfail == UNKNOWN or DONE)
            else:
                rb.sleep(0.001)
    except Robot_emo:
        print "Stopped by EMO"
        rb.close() # 로봇 Close
    except KeyboardInterrupt:
        print "Stopped by Ctrl+C"
        rb.close() # 로봇 Close
    finally:
        print 'Finished..'
        rb.close() # 로봇 Close
    
if __name__ == '__main__':
    main()


    