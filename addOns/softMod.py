'''
Save and load softMod membership to and from disk
'''
import pymel.core as pm
import json, os

def export_softMods():
    softies = [node for node in pm.ls() if node.hasAttr('isTSofty')]
    if not softies:
        return 'no softMods found in scene'
    softyDict = {}
    for softy in softies:
        geoList = pm.softMod(softy, q=1, g=1)
        softyDict[softy.name()] = geoList

    fileName = pm.fileDialog2(fileFilter='*.json', caption='Export soft mods to file')
    if fileName:
        with open(fileName[0], 'w') as outfile:
            json.dump(softyDict, outfile, sort_keys=True, indent=4, separators=(',', ': '))
        return 'Soft Mods saved successfully to: %s' % fileName[0]
    return 'Saving soft mods cancelled'

def buildFromFile(filename):
    '''
    Loads a json file created via the export_softMods function
    and adds the relevant geometry to each corresponding soft mod deformer in the scene
    Note this function does not create any new deformers
    '''
    if not os.path.exists(filename):
        return 'Soft Mods file not found: %s' % filename

    softyDict = {}

    with open(filename) as json_file:
        softyDict = json.load(json_file)

    for key in softyDict.keys():
        try:
            pm.softMod(key, e=1, g=softyDict[key])
        except:
            'unable to load membership for %s' % key


