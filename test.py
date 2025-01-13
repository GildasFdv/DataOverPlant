import pyvisa
rm = pyvisa.ResourceManager()
resources = rm.list_resources()
resource = resources[0]
spectrum_analyzer = rm.open_resource(resource)
print(spectrum_analyzer.query('*IDN?'))
# output: Rigol Technologies,DSA832E,DSA8H263000084,00.01.04.00.00

# Lire la fréquence centrale actuelle
center_frequency = spectrum_analyzer.query(':FREQ:CENT?')
print(f"Fréquence centrale : {center_frequency} Hz")

# Lire la largeur de bande actuelle
span = spectrum_analyzer.query(':FREQ:SPAN?')
print(f"Span actuel : {span} Hz")
