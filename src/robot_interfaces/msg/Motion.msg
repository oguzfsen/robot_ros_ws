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
