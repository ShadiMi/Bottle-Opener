"""
voice_utils.py

Handles voice recognition and async command listening for the simulation.
"""

import speech_recognition as sr

class VoiceCommandListener:
    """
    Thread-safe voice command recognizer.
    Use `start_listening()` to launch async listening.
    Access .last_command for latest recognized command.
    """
    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.mic = sr.Microphone()
        self.last_command = None
        self.listening = False

    def listen_for_command(self):
        with self.mic as source:
            print("Say 'open', 'big', 'small', 'rest', or 'reset'...")
            self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
            try:
                audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=3)
            except sr.WaitTimeoutError:
                print("Listening timed out, try again.")
                return ""
        try:
            text = self.recognizer.recognize_google(audio)
            print(f"You said: {text}")
            return text.lower()
        except sr.UnknownValueError:
            print("Didn't catch that.")
            return ""
        except sr.RequestError as e:
            print(f"Speech recognition error: {e}")
            return ""

    def listen_thread(self):
        import threading
        def _thread_func():
            self.listening = True
            cmd = self.listen_for_command()
            if cmd:
                self.last_command = cmd
            self.listening = False
        t = threading.Thread(target=_thread_func)
        t.daemon = True
        t.start()
