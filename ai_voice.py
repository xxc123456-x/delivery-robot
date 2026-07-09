import re, json, threading, queue

try:
    import pyttsx3; import pyaudio; import numpy as np
except ImportError as e:
    print(f"缺少依赖: {e}")
    print("请执行: pip install pyttsx3 pyaudio numpy openai-whisper")
    exit(1)


class AIVoice:
    def __init__(self, model_size='tiny'):
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 160); self.engine.setProperty('volume', 0.9)

        self.room_map = {
            '301': (2.5, 3.0), '302': (5.0, 3.0), '303': (-2.0, 3.0),
            '201': (2.5, -3.0), '202': (5.0, -3.0), '203': (-2.0, -3.0),
            '101': (2.5, 1.0), '102': (5.0, 1.0), '103': (-2.0, 1.0),
        }
        self.aliases = {'原点': (0,0), '充电桩': (0,0), '门口': (0,0), '起点': (0,0)}

        self.goal_callback = None; self.running = False

        print(f'[AI Voice] 加载 whisper {model_size}...')
        try:
            import whisper
            self.model = whisper.load_model(model_size)
            print('[AI Voice] Whisper 就绪 (本地)')
        except ImportError:
            print('[AI Voice] whisper未安装, 请: pip install openai-whisper')
            self.model = None

    def set_nav_callback(self, cb): self.goal_callback = cb

    def speak(self, text):
        print(f'[语音输出] {text}')
        self.engine.say(text); self.engine.runAndWait()

    def listen(self):
        """麦克风录音 → Whisper 本地转文字"""
        CHUNK, FORMAT, CHANNELS, RATE, RECORD_SEC = 1024, pyaudio.paInt16, 1, 16000, 5
        audio = pyaudio.PyAudio()
        stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)
        print('[语音输入] 正在听(5秒)...')
        frames = [stream.read(CHUNK) for _ in range(0, int(RATE/CHUNK*RECORD_SEC))]
        stream.stop_stream(); stream.close(); audio.terminate()

        data = np.frombuffer(b''.join(frames), dtype=np.int16).astype(np.float32)/32768.0

        if self.model:
            result = self.model.transcribe(data, language='zh', fp16=False)
            text = result['text'].strip()
        else:
            # 回退到 Google 在线 (需要联网)
            import speech_recognition as sr
            r = sr.Recognizer()
            try: text = r.recognize_google(sr.AudioData(data.tobytes(), RATE, 2), language='zh-CN')
            except: return None

        print(f'[语音输入] {text}')
        return text

    def parse_command(self, text):
        if not text: return None, None, None
        text = text.lower().replace(' ', '')

        # 房间号: 301, 302...
        m = re.search(r'(\d{3})', text)
        if m and m.group(1) in self.room_map:
            r = m.group(1); x, y = self.room_map[r]; return x, y, f'房间{r}'

        # 坐标: x2.5y1.5 或 2.5,1.5
        m = re.search(r'x([-]?\d+\.?\d*)y([-]?\d+\.?\d*)', text)
        if not m: m = re.search(r'([-]?\d+\.?\d*)[,，]([-]?\d+\.?\d*)', text)
        if m: return float(m.group(1)), float(m.group(2)), f'坐标({m.group(1)},{m.group(2)})'

        # 别名: 原点, 充电...
        for alias, pos in self.aliases.items():
            if alias in text: return pos[0], pos[1], alias

        return None, None, None

    def run_loop(self):
        self.running = True; self.speak('配送机器人就绪')
        while self.running:
            text = self.listen()
            if text and ('退出' in text or '停止' in text): self.speak('关机'); break
            x, y, desc = self.parse_command(text)
            if x is not None: self.speak(f'{desc}, 出发'); self.goal_callback and self.goal_callback(x, y)
            elif text: self.speak('请再说一次')

    def stop(self): self.running = False


if __name__ == '__main__':
    av = AIVoice(model_size='tiny')
    for t in ["去302", "回原点", "x1.5y2.0"]:
        x, y, d = av.parse_command(t); print(f"  '{t}' → {d}")
