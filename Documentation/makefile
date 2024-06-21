##
# UAVLAB Documentation PDF makefile
#

# end

pth:
	sed -i 's/* Table of Contents :toc:/* Table of Contents/' UAVLAB-serial-pth-documentation.org
	pandoc UAVLAB-serial-pth-documentation.org -o UAVLAB-serial-pth-documentation.pdf --template eisvogel --listings
	sed -i 's/* Table of Contents/& :toc:/' UAVLAB-serial-pth-documentation.org

i2c:
	sed -i 's/* Table of Contents :toc:/* Table of Contents/' UAVLAB-i2c-5hole-probe-documentation.org
	pandoc UAVLAB-i2c-5hole-probe-documentation.org -o UAVLAB-i2c-5hole-probe-documentation.pdf --template eisvogel --listings
	sed -i 's/* Table of Contents/& :toc:/' UAVLAB-i2c-5hole-probe-documentation.org

anem:
	sed -i 's/* Table of Contents :toc:/* Table of Contents/' UAVLAB-sonic-anemometer-documentation.org
	pandoc UAVLAB-sonic-anemometer-documentation.org -o UAVLAB-sonic-anemometer-documentation.pdf --template eisvogel --listings
	sed -i 's/* Table of Contents/& :toc:/' UAVLAB-sonic-anemometer-documentation.org
