##Trajektorija

    Članovi: Luka, Iva

    Ova sekcija projekta bavi se trajektorijom letjelice. Zasad se ovaj tim bavi proučavanjem dostupnih materijala i učenjem programa za izradu trajektorije.

    U budućnosti projekta, glavni će nam zadatak biti promjena i kompenzacija puta letenja u trenutku kada na letjelicu bude ugrađen novi gripper.

##(05.11.2024.) - Luka

    Push-anje ovog file-a na Git
    Sastanak cijelog tima, konačna podjela po sekcijama projekta

##(08.11.2024.) - Luka

    "Trajecotry" folder je pretvoren u ROS workspace i push-an, provjera od strane kolega bi dobro došla u slučaju da sam nešto napravio krivo
    Napravio da me naredba "roscd trajectory" automatski stavlja u taj workspace
    Još uvijek imam poteškoća s dockerom, ali uspio sam suzit izvor problema pa se nadam da cećo ga ubrzo moći riještit

##(11.12.2024.) - Luka

    Popravljena je greška u stvaranju ROS paketa za tim Trajectory (prvi put sam upotrijebio krivu naredbu za stvaranje paketa.)
    Docker i simulacija trenutno rade bez poteškoća.
    U svrhu vježbe i kasnijih implementacija planiram započeti pisati klasu koja će:
    	a) Stvoriti nasumičan objekt negdje u krugu robota (za početak samo singularna točka)
    	b) Pilotirati letjelicu do točke i pozicionirati ju tako da može uhvatiti gripperom (zasad uzimamo proizvoljnu veličinu grippera)
    
    Dodao sam prvu instancu TestFlight node-a.
    
##(19.12.2024.) - Luka

    Simulacija i docker rade svim članovima tima.
    Započeo sam s Python skrpitom koja će za zadanu točku generirati trajektoriju za letjelicu.
    Trenutno se ne obaziremo na postojanje grippera i letjelicu letimo do zadane točke.
    
##(22.12.2024.) - Luka

    Radio sam na skripti za generiranje trajektorije.
    Trenutna verzija iz 3 točke pronalazi jednadžbu 3D ravnine na kojoj će ležati naša parabola.
    Idući korak je pronalaženje parabole na toj ravnini.
    
##(29.12.2024.) - Luka

    Došlo je do promjene planova s moje strane.
    Isprava sam imao ideju generirati jednadžbu parabole iz 3 točke, na isti način kako bi se to napravilo matematički.
    Ali što sam više radio na tome, otkrio sam da mi je to preteško implementirati u Python kod.
    Novi plan je generirati listu waypoint-ova koji će činiti 'kvaziparabolu'.
    Danas sam radio na tom kodu, koji je sadržan u novom file-u 'make_trajectory.py'
    Komentari unutar koda objašnjavaju proces kojim trenutno stvaraom kvaziparabolu.
    Kad stignem, na GitHub ću staviti sliku i detaljnije objašnjenje koda.
    Skripta koja iz 3 točke daje jednadžbu ravnine još uvijek postoji, preimenovana u 'plane_from_3_points.py'.
    
