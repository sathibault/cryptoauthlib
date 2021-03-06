# CryptoAuthLib

This is a port of the Atmel
[CryptoAuthLib](http://www.atmel.com/tools/CryptoAuthLib.aspx) library
for the Arduino platform.

This is **beta** release, see disclaimer below.

## Installation

1. Download the CryptoAuthLib-v*xyz*.zip from the [latest release](https://github.com/sathibault/cryptoauthlib/releases)
2. From Arduino IDE menu select Sketch -> Include library -> Add .ZIP library and select the downloaded file.

## Examples

See provision_sha204 for an example of provisioning a device and using
it to computing challenge responses with symmetric keys with the
ATSHA204A chip.

See provision_ecc108 for an example of provisioning a device and using
it to generate private/public key pairs, signing messages, and
validating signed messages with the ATECC108A chip.

## License

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. The name of Atmel may not be used to endorse or promote products derived
   from this software without specific prior written permission.

4. This software may only be redistributed and used in connection with an
   Atmel integrated circuit.

THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
