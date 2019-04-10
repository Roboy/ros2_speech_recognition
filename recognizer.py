'''
Bing Speech To Text (STT) and Text To Speech (TTS)

based on https://github.com/Uberi/speech_recognition
'''

import json
import uuid
import wave
import io
from monotonic import monotonic
from urllib.parse import urlencode
from urllib.request import Request, urlopen
from urllib.error import URLError, HTTPError
import http.client

import webrtcvad
import collections
import sys
import signal
import pyaudio
import traceback

BING_KEY = 'e91e7a4512aa48b3b53b36b56bd1feb7'


class RequestError(Exception):
    pass


class UnknownValueError(Exception):
    pass


class LocaleError(Exception):
    pass


class BingVoice():
    def __init__(self, key):
        self.key = key
        # self.access_token = None
        # self.expire_time = None
        self.locales = {
            "ar-eg": {"Female": "Microsoft Server Speech Text to Speech Voice (ar-EG, Hoda)"},
            "de-DE": {"Female": "Microsoft Server Speech Text to Speech Voice (de-DE, Hedda)",
                      "Male": "Microsoft Server Speech Text to Speech Voice (de-DE, Stefan, Apollo)"},
            "en-AU": {"Female": "Microsoft Server Speech Text to Speech Voice (en-AU, Catherine)"},
            "en-CA": {"Female": "Microsoft Server Speech Text to Speech Voice (en-CA, Linda)"},
            "en-GB": {"Female": "Microsoft Server Speech Text to Speech Voice (en-GB, Susan, Apollo)",
                      "Male": "Microsoft Server Speech Text to Speech Voice (en-GB, George, Apollo)"},
            "en-IN": {"Male": "Microsoft Server Speech Text to Speech Voice (en-IN, Ravi, Apollo)"},
            "en-US": {"Female": "Microsoft Server Speech Text to Speech Voice (en-US, ZiraRUS)",
                      "Male": "Microsoft Server Speech Text to Speech Voice (en-US, BenjaminRUS)"},
            "es-ES": {"Female": "Microsoft Server Speech Text to Speech Voice (es-ES, Laura, Apollo)",
                      "Male": "Microsoft Server Speech Text to Speech Voice (es-ES, Pablo, Apollo)"},
            "es-MX": {"Male": "Microsoft Server Speech Text to Speech Voice (es-MX, Raul, Apollo)"},
            "fr-CA": {"Female": "Microsoft Server Speech Text to Speech Voice (fr-CA, Caroline)"},
            "fr-FR": {"Female": "Microsoft Server Speech Text to Speech Voice (fr-FR, Julie, Apollo)",
                      "Male": "Microsoft Server Speech Text to Speech Voice (fr-FR, Paul, Apollo)"},
            "it-IT": {"Male": "Microsoft Server Speech Text to Speech Voice (it-IT, Cosimo, Apollo)"},
            "ja-JP": {"Female": "Microsoft Server Speech Text to Speech Voice (ja-JP, Ayumi, Apollo)",
                      "Male": "Microsoft Server Speech Text to Speech Voice (ja-JP, Ichiro, Apollo)"},
            "pt-BR": {"Male": "Microsoft Server Speech Text to Speech Voice (pt-BR, Daniel, Apollo)"},
            "ru-RU": {"Female": "Microsoft Server Speech Text to Speech Voice (pt-BR, Daniel, Apollo)",
                      "Male": "Microsoft Server Speech Text to Speech Voice (ru-RU, Pavel, Apollo)"},
            "zh-CN": {"Female": "Microsoft Server Speech Text to Speech Voice (zh-CN, HuihuiRUS)",
                      "Female2": "Microsoft Server Speech Text to Speech Voice (zh-CN, Yaoyao, Apollo)",
                      "Male": "Microsoft Server Speech Text to Speech Voice (zh-CN, Kangkang, Apollo)"},
            "zh-HK": {"Female": "Microsoft Server Speech Text to Speech Voice (zh-HK, Tracy, Apollo)",
                      "Male": "Microsoft Server Speech Text to Speech Voice (zh-HK, Danny, Apollo)"},
            "zh-TW": {"Female": "Microsoft Server Speech Text to Speech Voice (zh-TW, Yating, Apollo)",
                      "Male": "Microsoft Server Speech Text to Speech Voice (zh-TW, Zhiwei, Apollo)"}
        }

        # authorize
        params = ""
        headers = {"Ocp-Apim-Subscription-Key": self.key}

        AccessTokenHost = "api.cognitive.microsoft.com"
        path = "/sts/v1.0/issueToken"

        # Connect to server to get the Access Token
        print ("Connect to server to get the Access Token")
        conn = http.client.HTTPSConnection(AccessTokenHost)
        start_time = monotonic()
        conn.request("POST", path, params, headers)
        response = conn.getresponse()
        print(response.status, response.reason)

        data = response.read()

        conn.close()

        self.access_token = data.decode("UTF-8")
        print ("Access Token: " + self.access_token)
        self.expire_time = start_time + 60*9


    def auth(self):

        if self.expire_time is None or monotonic() > self.expire_time:  # first credential request, or the access token from the previous one expired
            apiKey = self.key

            params = ""
            headers = {"Ocp-Apim-Subscription-Key": apiKey}

            #AccessTokenUri = "https://api.cognitive.microsoft.com/sts/v1.0/issueToken";
            AccessTokenHost = "api.cognitive.microsoft.com"
            path = "/sts/v1.0/issueToken"

            # Connect to server to get the Access Token
            print ("Connect to server to get the Access Token")
            conn = http.client.HTTPSConnection(AccessTokenHost)
            start_time = monotonic()
            conn.request("POST", path, params, headers)
            response = conn.getresponse()
            print(response.status, response.reason)

            data = response.read()

            conn.close()

            self.access_token = data.decode("UTF-8")
            print ("Access Token: " + self.access_token)
            self.expire_time = start_time + 60*9




    def recognize(self, audio_data, sample_rate, language="en-US", show_all=False):
        self.auth()
        wav_data = self.to_wav(audio_data, sample_rate)
        # self.save_wav(audio_data)

        url = "https://speech.platform.bing.com/recognize?{0}".format(urlencode({
            "version": "3.0",
            "requestid": uuid.uuid4(),
            "appID": "D4D52672-91D7-4C74-8AD8-42B1D98141A5",
            "format": "json",
            "locale": language,
            "device.os": "wp7",
            "scenarios": "ulm",
            "instanceid": uuid.uuid4()
        }))
        request = Request(url, data=wav_data, headers={
            "Authorization": "Bearer {0}".format(self.access_token),
            "Content-Type": "audio/wav; samplerate={0}; sourcerate={0}; trustsourcerate=true".format(sample_rate)
        })

        try:
            response = urlopen(request)
        except HTTPError as e:
            raise RequestError("recognition request failed: {0}".format(
                getattr(e, "reason", "status {0}".format(e.code))))  # use getattr to be compatible with Python 2.6
        except URLError as e:
            raise RequestError("recognition connection failed: {0}".format(e.reason))
        response_text = response.read().decode("utf-8")
        print(response_text)
        result = json.loads(response_text)

        # return results
        if show_all: return result
        if "header" not in result or "lexical" not in result["header"]: raise UnknownValueError()
        return result["header"]["lexical"]

    def synthesize(self, text, language="en-US", gender="Female"):
        self.auth()

        if language not in self.locales.keys():
            raise LocaleError("language locale not supported.")

        lang = self.locales.get(language)

        if gender not in ["Female", "Male", "Female2"]:
            gender = "Female"

        if len(lang) == 1:
            gender = lang.keys()[0]

        service_name = lang[gender]

        body = "<speak version='1.0' xml:lang='en-us'>\
                <voice xml:lang='%s' xml:gender='%s' name='%s'>%s</voice>\
                </speak>" % (language, gender, service_name, text)

        headers = {"Content-type": "application/ssml+xml",
                   "X-Microsoft-OutputFormat": "raw-16khz-16bit-mono-pcm",
                   "Authorization": "Bearer " + self.access_token,
                   "X-Search-AppId": "07D3234E49CE426DAA29772419F436CA",
                   "X-Search-ClientID": str(uuid.uuid1()).replace('-', ''),
                   "User-Agent": "TTSForPython"}

        url = "https://speech.platform.bing.com/synthesize"
        request = Request(url, data=body, headers=headers)
        try:
            response = urlopen(request)
        except HTTPError as e:
            raise RequestError("tts request failed: {0}".format(
                getattr(e, "reason", "status {0}".format(e.code))))  # use getattr to be compatible with Python 2.6
        except URLError as e:
            raise RequestError("tts connection failed: {0}".format(e.reason))

        data = response.read()



        p = pyaudio.PyAudio()

        stream = p.open(format=p.get_format_from_width(2),
                        channels=1,
                        rate=16000,
                        output=True)

        # TODO: change voice pitch

        # data = np.array(wave.struct.unpack("%dh"%(len(data)/2), data))*2
        # data = np.fft.rfft(data)
        # data = np.fft.irfft(data)
        # dataout = np.array(data*0.1, dtype='int16') #undo the *2 that was done at reading
        # chunkout = struct.pack("%dh"%(len(dataout)), *list(dataout)) #convert back to 16-bit data
        # stream.write(chunkout)

        stream.write(data)

        stream.stop_stream()
        stream.close()

        p.terminate()

        return data

    @staticmethod
    def save_wav(raw_data):
        with wave.open("test.wav", 'w') as output:
            output.setnchannels(1)
            output.setframerate(44000)
            output.setsampwidth(2)
            output.writeframes(raw_data)
            output.close()

    @staticmethod
    def to_wav(raw_data, sample_rate):
        # generate the WAV file contents
        with io.BytesIO() as wav_file:
            wav_writer = wave.open(wav_file, "wb")
            try:  # note that we can't use context manager, since that was only added in Python 3.4
                wav_writer.setframerate(sample_rate)
                wav_writer.setsampwidth(2)
                wav_writer.setnchannels(1)
                wav_writer.writeframes(raw_data)
                wav_data = wav_file.getvalue()
            finally:  # make sure resources are cleaned up
                # wav_file.write("test.wav")
                wav_writer.close()
        return wav_data


    def stt_with_vad(self, source, language="en-US"):

        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 16000
        CHUNK_DURATION_MS = 20  # supports 10, 20 and 30 (ms)
        PADDING_DURATION_MS = 1000
        CHUNK_SIZE = int(RATE * CHUNK_DURATION_MS / 1000)
        CHUNK_BYTES = CHUNK_SIZE * 2
        NUM_PADDING_CHUNKS = int(PADDING_DURATION_MS / CHUNK_DURATION_MS)
        NUM_WINDOW_CHUNKS = int(240 / CHUNK_DURATION_MS)

        vad = webrtcvad.Vad(1)
        got_a_sentence = False
        leave = False


        def handle_int(sig, chunk):
            global leave, got_a_sentence

            leave = True
            got_a_sentence = True

        signal.signal(signal.SIGINT, handle_int)
        global lock

        while not leave:
            ring_buffer = collections.deque(maxlen=NUM_PADDING_CHUNKS)
            triggered = False
            voiced_frames = []
            ring_buffer_flags = [0] * NUM_WINDOW_CHUNKS
            ring_buffer_index = 0
            buffer_in = ''

            print("* recording")
            # with lock:
            source.__enter__()
            # odas.channels[0].record = True

            while not got_a_sentence: #and not leave:
                chunk = source.stream.read(CHUNK_SIZE)
                active = vad.is_speech(chunk, RATE)
                sys.stdout.write('1' if active else '0')
                ring_buffer_flags[ring_buffer_index] = 1 if active else 0
                ring_buffer_index += 1
                ring_buffer_index %= NUM_WINDOW_CHUNKS
                if not triggered:
                    ring_buffer.append(chunk)
                    num_voiced = sum(ring_buffer_flags)
                    if num_voiced > 0.5 * NUM_WINDOW_CHUNKS:
                        sys.stdout.write('+')
                        triggered = True
                        voiced_frames.extend(ring_buffer)
                        ring_buffer.clear()
                else:
                    voiced_frames.append(chunk)
                    ring_buffer.append(chunk)
                    num_unvoiced = NUM_WINDOW_CHUNKS - sum(ring_buffer_flags)
                    if num_unvoiced > 0.9 * NUM_WINDOW_CHUNKS:
                        sys.stdout.write('-')
                        triggered = False
                        got_a_sentence = True

                sys.stdout.flush()

            sys.stdout.write('\n')
            data = b''.join(voiced_frames)

            source.__exit__(None, None, traceback)
            # odas.channels[0].record = False
            print("* done recording")

            # recognize speech using Microsoft Bing Voice Recognition
            try:
                # pdb.set_trace()
                text = self.recognize(data, language=language, sample_rate=source.SAMPLE_RATE)
                # pdb.set_trace()
                print('Bing:' + text)
                # stream.close()
                return text
            except UnknownValueError:
                traceback.print_exc()
                print("Microsoft Bing Voice Recognition could not understand audio")
            except RequestError as e:
                print("Could not request results from Microsoft Bing Voice Recognition service; {0}".format(e))

            got_a_sentence = False

        # stream.close()

        return text