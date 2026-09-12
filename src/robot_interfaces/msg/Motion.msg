std_msgs/Header header
bool arm
int32 throttle
int32 steering
bool reverse
# Hubmotor arm rolesinden bagimsiz olarak dumen motorlarini durdurur.
bool stop_steering
# True: gaz sifir olsa da fren acik kalir. Normal durus ve emergency'de False.
bool coast
# Komutun kaynagi; RC devraldiginda otonom komutlar uygulanmaz.
bool autonomous
# 0: normal surus, -1: eksenel sola (CCW), +1: eksenel saga (CW).
int8 pivot
