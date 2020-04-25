import json
import pymel.core as pm
import os


##----------------------------------------------------------------------------------------------------------------------
#
# SAVING WEIGHTS
#
##----------------------------------------------------------------------------------------------------------------------
def getSkin(node):
    skin = None
    skin = [x for x in pm.listHistory(node) if pm.nodeType(x) == "skinCluster"]
    if skin:
        return skin[0]
    else:
        return None

def getFfdSkinWeights(ffd):
    skin = getSkin(ffd)
    if not skin:
        return "No skinCluster found on %s" % ffd.name()

    print 'skinCluster found: %s' % skin
    skinDict = {'node': ffd.name(),
                'skinCluster': skin.name(),
                'infs': [inf.name() for inf in pm.skinCluster(skin, q=1, inf=1)],
                'points': {}}

    for point in ffd.pt:
        values = pm.skinPercent(skin, point, value=1, q=1)
        skinDict['points'][point.name()] = {}
        for index, value in enumerate(values):
            if value > 0:
                skinDict['points'][point.name()][skinDict['infs'][index]] = value

    return skinDict

def loadFfdSkinWeights(skinFile):
    if not os.path.exists(skinFile):
        return 'Skin file not found: %s' % skinFile

    skinDict = {}

    with open(skinFile) as json_file:
        skinDict = json.load(json_file)

    ffd = pm.PyNode(skinDict['node'])
    infs = [pm.PyNode(inf) for inf in skinDict['infs']]

    skinCluster = pm.skinCluster([ffd] + infs, name=skinDict['skinCluster'])

    for point in skinDict['points'].keys():
        valuesList = []
        p = skinDict['points'][point]
        for inf in p.keys():
            valuesList.append((inf, p[inf]))

        pm.skinPercent(skinCluster, point, tv=valuesList)

    return 'Skin weights loaded successfully onto %s' % ffd.name()



def saveWeights(skinDict):
    fileName = pm.fileDialog2(fileFilter='*.json', caption='Save Skin Weights to file')
    if fileName:
        with open(fileName[0], 'w') as outfile:
            json.dump(skinDict, outfile, sort_keys=True, indent=4, separators=(',', ': '))
        return 'Skin weights saved successfully to: %s' % fileName[0]
    return 'Saving skin weights cancelled'
