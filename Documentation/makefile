##
# UAVLAB Documentation
#
# @file
# @version 0.1

# end

serial:
	sed -i 's/* Table of Contents :toc:/* Table of Contents/' UAVLAB-serial-pth-documentation.org
	pandoc UAVLAB-serial-pth-documentation.org -o UAVLAB-serial-pth-documentation.pdf --template eisvogel --listings
	sed -i 's/* Table of Contents/& :toc:/' UAVLAB-serial-pth-documentation.org

i2c:
	sed -i 's/* Table of Contents :toc:/* Table of Contents/' UAVLAB-i2c-tca-documentation.org
	pandoc UAVLAB-i2c-tca-documentation.org -o UAVLAB-i2c-tca-documentation.pdf --template eisvogel --listings
	sed -i 's/* Table of Contents/& :toc:/' UAVLAB-i2c-tca-documentation.org
