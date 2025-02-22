# Trajektorija
Članovi: Luka, Iva

Ova sekcija projekta bavi se trajektorijom letjelice.

Glavni će nam je zadatak promjena i kompenzacija puta letenja u trenutku kada na letjelicu bude ugrađen novi gripper.

## (05.11.2024.) - Luka

    Push-anje ovog file-a na Git
    Sastanak cijelog tima, konačna podjela po sekcijama projekta

## (08.11.2024.) - Luka

    "Trajecotry" folder je pretvoren u ROS workspace i push-an, provjera od strane kolega bi dobro došla u slučaju da sam nešto napravio krivo
    Napravio da me naredba "roscd trajectory" automatski stavlja u taj workspace
    Još uvijek imam poteškoća s dockerom, ali uspio sam suzit izvor problema pa se nadam da cećo ga ubrzo moći riještit

## (11.12.2024.) - Luka

    Popravljena je greška u stvaranju ROS paketa za tim Trajectory (prvi put sam upotrijebio krivu naredbu za stvaranje paketa.)
    Docker i simulacija trenutno rade bez poteškoća.
    U svrhu vježbe i kasnijih implementacija planiram započeti pisati klasu koja će:
    	a) Stvoriti nasumičan objekt negdje u krugu robota (za početak samo singularna točka)
    	b) Pilotirati letjelicu do točke i pozicionirati ju tako da može uhvatiti gripperom (zasad uzimamo proizvoljnu veličinu grippera)
    
    Dodao sam prvu instancu TestFlight node-a.
    
## (19.12.2024.) - Luka

    Simulacija i docker rade svim članovima tima.
    Započeo sam s Python skrpitom koja će za zadanu točku generirati trajektoriju za letjelicu.
    Trenutno se ne obaziremo na postojanje grippera i letjelicu letimo do zadane točke.
    
## (22.12.2024.) - Luka

    Radio sam na skripti za generiranje trajektorije.
    Trenutna verzija iz 3 točke pronalazi jednadžbu 3D ravnine na kojoj će ležati naša parabola.
    Idući korak je pronalaženje parabole na toj ravnini.
    
## (29.12.2024.) - Luka

    Došlo je do promjene planova s moje strane.
    Isprava sam imao ideju generirati jednadžbu parabole iz 3 točke, na isti način kako bi se to napravilo matematički.
    Ali što sam više radio na tome, otkrio sam da mi je to preteško implementirati u Python kod.
    Novi plan je generirati listu waypoint-ova koji će činiti 'kvaziparabolu'.
    Danas sam radio na tom kodu, koji je sadržan u novom file-u 'make_trajectory.py'
    Komentari unutar koda objašnjavaju proces kojim trenutno stvaraom kvaziparabolu.
    Kad stignem, na GitHub ću staviti sliku i detaljnije objašnjenje koda.
    Skripta koja iz 3 točke daje jednadžbu ravnine još uvijek postoji, preimenovana u 'plane_from_3_points.py'.
    
## (30.12.2024.) - Luka

    Nakon još par pokušaja, ipak sam uspio napisati kod za trajektoriju koristeći jednadžbu parabolu.
    Trenutno rješenje stvara 3. točku (koja nije početna i krajnja) te pomoću dvije polu-parabole stvara waypoints[] listu.
    Potrebno je još testiranja koda kako bi se osiguralo da radi za sve slučajeve.
    U nekom ću trenutku napraviti dokument gdje se objašnjava trenutni pristup stvaranju trajektorije, kao i moguće izmjene.
    Također, vjerujem da je u kodu moguće puno opće optimizacije (u svrhu brzine.)
    Od Vision team-a nam, zasad, treba samo koordinata točke gdje želimo da letjelica sleti.
    
    - Dodatak:
    Dodao sam PDF s detaljima postupka nalaženja trajektorije.
    Dodao sam još jedan TODO u kod vezan uz slučaj spuštanja na točku.
    
## (5.1.2025.) - Luka
    
    Malo sam doradio kod za stvaranje trajektorije.
    Sada se density waypoint-ova može birati proizoljnim, pozitivnim integerom.
    Također sam dodao flag za slučaj da je target point niže od početne točke.
    U tom slučaju bi teoretski bila dosta jedna polu-parabola, što još treba implementirati, ali nije prvi prioritet.
    Trenutni glavni zadatak je napraviti da naša skripta može poslati waypoints polje skirpit trajectory_ros_testing.py.
    Osim toga, potrebno je napraviti transformaciju koordinata unutar make_trajectory.py skripte, tako da su koordinate letjelice uvijek (0,0,0).
    Konačno, trebamo informacije o tome što da radimo vezano za ostale parametre waypoints polja (rotacija, brzina, akceleracija).
    
## (18.1.2025.) - Luka

    Napravio sam novu skrpitu 'simulation_test.py' kojom ćemo testirati u simulaciji.
    Skrpita je spoj postojećih skripti 'make_trajectory.py' i 'trajectory_ros_testing.py'
    Trenutna verzija radi, u smislu da se može izvesti u simulaciji i letjelica će pratiti zadane waypoints.
    Problem je što rezultati trenutno nisu onakvi kakve sam očekivao, letjelica ima čudne kretnje i nekonzistentnu putanju za iste waypoints.
    Imam par ideja gdje bi mogao bit problem:
    	- Način na koji se koordinate transofrmiraju iz simulacije u "perspektivu letjelice" unutar 'waypoints_generator' funkcije
    	- Nedostatak ostalih parametera unutar waypoint-ova (brzina, akceleracija, kutna brzina)
    	- Način na koji se waypoints lista stvara u programu i predaje letjelici
    Rješavanje ovih problema je trenutni prioritet trajectory tima.
    Nakon toga, potrebno je napraviti da ciljnu točku možemo birati sami, bez da je moramo tvrdo kodirati.
    Trenutno, idealno rješenje za to bila bi još jedna klasa koja publish-a na zaseban topic, zato što ćemo ubuće točku dobivati od vision tima.
    
## (19.1.2025.) - Luka

    Mislim da sam otkrio problem iz priješnjeg update-a.
    Unutar originalne skripte 'make_trajectory.py' sam napravio katastrofalnu grešku zbog koje se waypoints nisu dobro računali u slučaju kad je x i y smjer negativan.
    Taj problem je sad otkolnjen, i skripta uspješno izbacuje waypoints za sve 4 moguće kombinacije (x,y) kretanja.
    Također sam prepravio skriptu za simulaciju za isti problem.
    Sad ću krenuti s testiranjem u simulaciji.
    Ako sve radi kako treba, na Teams-u bi trebao biti video.
    
## (18.1.2025.) - Iva

    Dron može poletjeti i sletjeti po trajektoriji elipse. Skripta spojena na simulaciju. 
    Skripta se zove follow_elliptical_coordinates.py
    Potrebno je još napisati subscriber za sljedece stvari:
    - subscriber za trenutnu poziciju drona (tako da se trajektorija moze racunati u odnosu na to, a ne hardkodirano)
    - subscriber za primanje sljedece tocke, tako da se trajektorija racuna u odnosu na to.
      Na ovaj topic bismo se trebali povezati s timom Vision.
    Sto napraviti s brzinom, akceleracijom i rotacijom?

## (19.1.2025.) - Iva

    Dodan subscriber za target i za trenutnu poziciju.
    Popravljene neke greske.
    Treba:
    - Spojjiti s coordinate converter Vision tima
    - jednom kad to proradi, provjeriti radi li generiranje trajektorije u elipsi ispravno
    
## (22.2.2025.) - Luka

    Popravljena make_trajectory.py skripta tako da radi u svim smjerovima i za bilo koje točke cilja i pozicije.
