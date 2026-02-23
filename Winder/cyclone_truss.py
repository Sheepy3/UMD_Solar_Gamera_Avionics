from pathlib import Path
import json
import math

scritpDir = Path(__file__).resolve().parent

def loadJsonFile(filepath: str) -> dict:
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            data = json.load(f)
        return data
    except FileNotFoundError:
        print(f"Error: The file '{filepath}' was not found.")
        return None
    except json.JSONDecodeError:
        print(f"Error: Could not decode JSON from '{filepath}'. Check file format.")
        return None
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return None

def areCoprime(a: int, b: int) -> bool:
  return math.gcd(a, b) == 1

if __name__ == '__main__':
    trussSettings = loadJsonFile(scritpDir / "trussSettings.json")
    trussParameters = trussSettings.get("trussParameters")
    windingParameters = trussSettings.get("windingParameters")

    if not trussSettings or not trussParameters or not windingParameters:
        print("Failed to load Truss Settings.")
        exit(-1)
    else:
        print("Truss Settings Loaded Successfully")
        
    #Check if values are intergers
    if not isinstance(trussParameters.get("numChords"), int):
        print(f"Error: numChords must be an integer.")
        exit(-1)
        
    for param in ["numNodes", "numLayers", "skipIndex"]:
        if not isinstance(windingParameters.get(param), int):
            print(f"Error: {param} must be an integer.")
            exit(-1)
            
    if not isinstance(trussSettings.get("lockParameters").get("lockWindings"), int):
        print("Error: lockWindings must be an integer.")
        exit(-1)
        
    #Check if chord and skipIndex are coprime
    if not areCoprime(trussParameters.get("numChords"), windingParameters.get("skipIndex")):
        print("Error: numChords and skipChords must be coprime.")
        exit(-1)

    towRequired = 0
    rotationAngle = 0
    
    #Calculate the degrees of rotation and tow required for locks
    lockDegrees = 360 * trussSettings["lockParameters"]["lockWindings"]
    lockTowRequired = trussParameters.get("chordSpacing") * trussParameters.get("numChords") * trussSettings["lockParameters"]["lockWindings"]
    
    #Calculate number of rotations of truss to get number of nodes
    numRotations = windingParameters.get("numNodes") / trussParameters.get("numChords")
    degreesRotations = 360 * numRotations
    
    #Calculate rotations rounded up to nearest whole rotation
    extraRotation = math.ceil(numRotations) - numRotations
    degreesExtraRotation = 360 * extraRotation
    
    #Calculate skip rotation and degrees
    skipRotation = windingParameters.get("skipIndex") / trussParameters.get("numChords")
    degreesSkipRotation = 360 * skipRotation
    
    #Calculate lengths of each winding component
    unwoundLength = windingParameters.get("numNodes") * trussParameters.get("chordSpacing")
    chordLength = trussParameters.get("chordLength")
    
    #Calculate angle of winding
    angle = math.atan(chordLength / unwoundLength) * (180 / math.pi)
    
    #Calculate tow required for each winding component
    windTowRequired = math.sqrt(chordLength ** 2 + unwoundLength ** 2)
    extraTowRequired = trussParameters.get("chordSpacing") * trussParameters.get("numChords") * extraRotation
    skipTowRequired = trussParameters.get("chordSpacing") * trussParameters.get("numChords") * skipRotation

    with open(scritpDir / f"{trussSettings["outputFileName"]}.gcode", "w", encoding="utf-8") as gcodeOutput:
        gcodeOutput.write("G1 X0 Y0 Z0\n")
        gcodeOutput.write(f"G1 F{trussSettings["defaultFeedRate"]}\n")

        for layer in range(windingParameters.get("numLayers")):
            for startingChord in range(trussParameters.get("numChords")):
                gcodeOutput.write(f"; Layer {layer + 1}/{windingParameters.get("numLayers")}, Chord {startingChord + 1}/{trussParameters.get("numChords")}, Circuit 1/2\n")

                #Create lock
                gcodeOutput.write(f"G1 Y{rotationAngle + lockDegrees}\n")
                
                rotationAngle += lockDegrees
                towRequired += lockTowRequired

                #Wind around chords forward
                gcodeOutput.write(f"G1 Z{-angle}\n")
                gcodeOutput.write(f"G1 X{chordLength} Y{rotationAngle + degreesRotations}\n")
                
                rotationAngle += degreesRotations
                towRequired += windTowRequired
                
                #Complete extra rotation
                gcodeOutput.write(f"G1 Z0\n")
                gcodeOutput.write(f"G1 Y{rotationAngle + degreesExtraRotation}\n")
                
                rotationAngle += degreesExtraRotation
                towRequired += extraTowRequired
                
                gcodeOutput.write(f"; Layer {layer + 1}/{windingParameters.get("numLayers")}, Chord {startingChord + 1}/{trussParameters.get("numChords")}, Circuit 2/2\n")
                
                #Create lock
                gcodeOutput.write(f"G1 Y{rotationAngle + lockDegrees}\n")
                
                rotationAngle += lockDegrees
                towRequired += lockTowRequired
                
                #Wind around chords forward                
                gcodeOutput.write(f"G1 Z{angle}\n")
                gcodeOutput.write(f"G1 X0 Y{rotationAngle + degreesRotations}\n")
                
                rotationAngle += degreesRotations
                towRequired += windTowRequired
                
                #Complete extra rotation
                gcodeOutput.write(f"G1 Z0\n")
                gcodeOutput.write(f"G1 Y{rotationAngle + degreesExtraRotation}\n")
                
                rotationAngle += degreesExtraRotation
                towRequired += extraTowRequired
                
                #Skip chrods for next winding
                gcodeOutput.write(f"G1 Y{rotationAngle + degreesSkipRotation}\n")
                
                rotationAngle += degreesSkipRotation
                towRequired += skipTowRequired
                
        #Add final lock
        gcodeOutput.write(f"G1 Y{rotationAngle + lockDegrees}\n")
        
        rotationAngle += lockDegrees
        towRequired += lockTowRequired
        
    print(f"G-code generation complete. Estimated tow required: {towRequired / 1000:.2f} meters")