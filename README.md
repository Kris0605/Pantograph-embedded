# Haptikus pantográf beágyazott szoftvere PIC24 mikrovezérlőre

Készítette Riskutia Balázs a "Pantográf alapú kinesztetikus haptikus eszköz tervezése" című szakdolgozatának keretein belül, illetve Honti Kristóf a "Virtuális környezet fejlesztése kinesztetikus haptikus eszköz felhasználásával" című szakdolgozatának keretein belül, a BME MOGI Tanszéken.

## A kódbázisról röviden

Jelen kódbázis egy MPLAB X IDE projektet alkot, amely a Microchip gyártónak a PIC24 mikrovezérlőt is támogató fejlesztői környezete. Fordításához az XC16 C fordítóprogram szükséges, melyet az MPLAB nem tartalmaz alapértelmezetten. A kód tömegének jelentős részét a tanszéki uMogi fejlesztőkártya használatához szükséges konfigurációk, illetve függvények alkotják. Ide tartozik az USB-n keresztüli virtuális soros komminukációt megvalósító kód is. A pantográf szempontjából releváns elemeket a következő szakasz ismerteti.

## Felépítés

Ez a szakasz tárgyalja a pantográf, és a virtuális tér működtetése szempontjából releváns programfájlokat.

### Főprogram

A `main.c` tartalmazza a kezdeti konfigurációt, definiálja a használt interruptokat, felelős az enkóderek lekérdezéséért, a mechanikai számítások futtatásáért, illetve a motorok aktivizálásáért. Az enkóderek és a motorok kezelése közvetlenül itt található meg.

### Mechanikai szimuláció

A `mechanics.h` és a `mechanics.c` fájlok tartalmazzák a mechanikai szimulációhoz szükséges változókat, segédváltozókat, illetve függvényeket. Kitüntetett szerepet kap a `mechanics` és a `spinner_mechanics` struktúra, mely a virtuális térben mozgó objektumok mechanikai tulajdonságait, illetve állapotváltozóinak aktuális értékeit tárolja. Jelenleg az end-effectorhoz, és a virtuális golyóhoz tartozik egy-egy `mechanics` struktúra-objektum, illetve a forgó tárgyakhoz a `spinner_mechanics` struktúra-objektum.

### Soros kommunikáció

A soros kommunikáció során küldendő adatok formátuma és módja az `app_device_cdc_basis.c` fájlban van meghatározva.
