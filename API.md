ROS 노드 초기화 : SDK를 사용하기 전에 ROS 노드를 초기화해야 합니다. 이는 rospy.init_node()`init()` 메서드를 사용하여 수행

토픽 구독자 또는 게시자 생성 : 필요에 따라 토픽에 대한 구독자 또는 게시자를 생성합니다. 구독자는 토픽 데이터를 수신하는 데 사용되고, 게시자는 제어 명령을 전송

서비스 프록시 생성 : 호출해야 하는 서비스에 대해 서비스 프록시 객체를 생성하고, 해당 객체를 통해 요청을 보내고 응답

데이터 처리 또는 응답 : 토픽 데이터 또는 서비스 응답을 수신하면 해당 처리 또는 작업을 수행

--------------------------------------------------------------------
`/get_used_audio_buffer_size` 는 서비스는 오디오 재생 버퍼 사용량에 대한 정보를 얻는 데 사용됩니다.

요청 형식
유형 :Trigger
응답 형식
유형 :TriggerResponse
분야 :
`success(bool)`: 요청 성공 여부를 나타냅니다. True0은 성공, False0은 실패를 의미합니다.
`message(str)`: 오디오 재생 버퍼 사용량(현재 큐에 있는 오디오 블록 수)을 반환합니다.  

--------------------------------------------------------------------

`/get_used_audio_buffer_size`  는 서비스는 오디오 재생 버퍼 사용량에 대한 정보를 얻는 데 사용됩니다.

요청 형식
유형 :Trigger
응답 형식
유형 :TriggerResponse
분야 :
`success(bool)`: 요청 성공 여부를 나타냅니다. True0은 성공, False0은 실패를 의미합니다.
`message(str)`: 오디오 재생 버퍼 사용량(현재 큐에 있는 오디오 블록 수)을 반환합니다.

--------------------------------------------------------------------
`/play_music` 은 서비스는 로봇을 제어하여 지정된 음악 파일을 재생하는 데 사용됩니다. 사용자는 음악 파일의 이름이나 번호, 그리고 볼륨을 입력하여 음악 재생을 활성화할 수 있습니다.

요청 형식
유형 :playmusicRequest
분야 :
music_number(str): 음악 파일의 이름 또는 번호입니다. 파일 경로이거나 미리 정의된 음악 번호일 수 있습니다.
volume(정수): 음악의 볼륨으로, 일반적으로 0에서 100 사이의 값을 가집니다.
응답 형식
유형 :playmusicResponse
분야 :
`success_flag(bool)`: 음악 재생 요청이 성공했는지 여부를 나타냅니다. True0은 성공, False0은 실패를 의미합니다.

사용 예시  
```python  
import rospy
from kuavo_msgs.srv import playmusic, playmusicRequest

rospy.init_node('music_player_client')

robot_music_play_client = rospy.ServiceProxy("/play_music", playmusic)

request = playmusicRequest()
request.music_number = "/home/kuavo/.."  
request.volume = 80  

response = robot_music_play_client(request)
```
