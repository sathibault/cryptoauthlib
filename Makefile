all: CryptoAuthLib.zip

.PHONY: CryptoAuthLib.zip
CryptoAuthLib.zip:
	-rm CryptoAuthLib.zip
	-rm -rf release
	mkdir -p release
	cp -r library.properties src examples release
	cd release; zip -r ../CryptoAuthLib.zip * --exclude '*~'
