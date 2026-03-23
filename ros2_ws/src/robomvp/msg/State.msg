# Wiadomość reprezentująca aktualny stan automatu stanowego
# Publikowana co krok timera (domyślnie co 1 sekundę)
# Przykładowe wartości state_name: SEARCH_TABLE, PICK_BOX, FINISHED
# state_id odpowiada wartości enuma State (0=SEARCH_TABLE ... 7=FINISHED)

string state_name
int32 state_id
