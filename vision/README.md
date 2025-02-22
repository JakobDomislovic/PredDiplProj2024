# Vision team

* Članovi: Marija, Toma
* Ovaj tim je zadužen za detekciju ArUco markera.


## ChangelogDo

* dodani paketi za detekciju, jedan gdje je dron statičan, drugi gdje se pozicija drona dobiva optitrack-om

## (21.02.2025.) - Luka D.

* Bilo je potrebno izbrisati cijeli folder "static drone". Naime, taj folder i optitrack folder su oboje imali pakete imena "transforms", što je uzrokovalo teško uklonjivu grešku prilikom korištenja ros-a (ros core se nije mogao upaliti). S obzirom da je za projekt predviđeno korištenje optitracka, ostavio sam taj paket, dok je "static drone" uklonjen iz glavne verzije. U slučaju da bude potreban, izbrisani paket se nalazi na GitHub-u među prošlim verzijama repozitorija te sam ga lokalno spremio za svaki slučaj. Svjedeno bi bilo pametno preimenovati paket u nešto drugačije, jer samo "transforms" može stvoriti dodatne zabude.
