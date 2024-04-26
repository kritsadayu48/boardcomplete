#include <Arduino.h>
#include <ATD3.5-S3.h>
#include <Audio.h> // require install ESP32-audioI2S lib

Audio audio;

void setup() {
    Serial.begin(115200);
    
    Sound.begin();
    Card.begin();

    audio.setVolume(7); // can set 0 to 21

    audio.connecttoFS(Card, "/test.mp3"); // play test.mp3 in MicroSD Card
}

void loop() {
  audio.loop(); // keep MP3 decoder and Codec work
}

// for debug only, can remove if not use
void audio_info(const char *info){
  Serial.print("info        "); Serial.println(info);
}

void audio_id3data(const char *info){  //id3 metadata
   Serial.print("id3data     ");Serial.println(info);
}

void audio_eof_mp3(const char *info){  //end of file
  Serial.print("eof_mp3     ");Serial.println(info);
}

void audio_showstation(const char *info){
  Serial.print("station     ");Serial.println(info);
}

void audio_showstreamtitle(const char *info){
  Serial.print("streamtitle ");Serial.println(info);
}

void audio_bitrate(const char *info){
  Serial.print("bitrate     ");Serial.println(info);
}

void audio_commercial(const char *info){  //duration in sec
  Serial.print("commercial  ");Serial.println(info);
}

void audio_icyurl(const char *info){  //homepage
  Serial.print("icyurl      ");Serial.println(info);
}

void audio_lasthost(const char *info){  //stream URL played
  Serial.print("lasthost    ");Serial.println(info);
}
