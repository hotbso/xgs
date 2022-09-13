# xgs landing speed plugin for X-plane
x-plane xgs landing speed plugin - reloaded

This is a further development of the well known "Landing speed plugin"

The original code contains compiled in limits and text for rating the landing that are not appropriate for airliners.
There are some mods on the net with other compiled in limits.

In addition to the 'old' xgs plugin this version features:

- per aircraft configuration files that are easily editable.
  A sample configuration file for the Airbus A320 family is included.

- Reporting
  - height above threshold
  - touchdown distance from threshold
  - deviation from center line
  - IAS / VLS and pitch at toutchdown

- g-force is computed as derivative of vertical speed so these two values really match

Home page on xplane.org: https://forums.x-plane.org/index.php?/files/file/45734-landing-speed-plugin-xgs-reloaded/

The latest binary kit can always be found here: https://github.com/hotbso/xgs/releases

Thanks to
- Saso Kiselkov for his fantastic libacfutils library https://github.com/skiselkov/libacfutils !
- hoempapaaa for providing support for MacOS https://forums.x-plane.org/index.php?/profile/703701-hoempapaaa
- Rodeo314 for providing support for MacOS https://github.com/Rodeo314

Note:
Saso Kiselkov no longer updates the precompiled redist- kit of libacfutils and the latest one is not compatible with XP12.
Therefore this repo contains a small subset of libacfutils of the above mentioned repo (commit
7e25f911d504b002203825a0af39c5f8cdc818d0) needed for airportdb .
Only conf.c was replaced with an older version that does not depend on CURL which is a pain to provide on mingw64.

