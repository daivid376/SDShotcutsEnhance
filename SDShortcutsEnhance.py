#__author__ = daiwei
#version v1.00
import sd,os,json,math
from PySide2 import QtGui,QtWidgets,QtCore


from sd.api.sdbasetypes import float2
from  sd.api.sdvalueint import SDValueInt
from  sd.api.sdvalueint2 import SDValueInt2
from  sd.api.sdvalueint3 import SDValueInt3
from  sd.api.sdvalueint4 import SDValueInt4
from sd.api.sdvaluefloat import SDValueFloat
from sd.api.sdvaluefloat2 import SDValueFloat2
from sd.api.sdvaluefloat3 import SDValueFloat3
from sd.api.sdvaluefloat4 import SDValueFloat4
from sd.api.sdvaluebool import SDValueBool
from sd.api.sdvaluebool2 import SDValueBool2
from sd.api.sdvaluebool3 import SDValueBool3
from sd.api.sdvaluebool4 import SDValueBool4
from sd.api.sdtypeenum import SDTypeEnum
from sd.api.sdvaluestring  import SDValueString
from sd.api.sdproperty import SDPropertyCategory,SDPropertyInheritanceMethod
from sd.api.sdapplication import SDApplicationPath
from sd.api.sdgraphobjectframe import SDGraphObjectFrame

from sd.api.sdbasetypes import int2, int3, int4, float2, float3, float4, bool2, bool3, bool4, ColorRGBA


def getUIMgr():
    context = sd.getContext()
    app = context.getSDApplication()
    UIMgr = app.getQtForPythonUIMgr()
    return (UIMgr)

class ShortcutsFunctions():
    def __init__(self):
        self.context = sd.getContext()
        self.app = self.context.getSDApplication()
        self.UIMgr = getUIMgr()

        self.mainWindow = self.UIMgr.getMainWindow()

        self.currentGraph = None
        self.sels = None

        self.functions = [{"key":'c','function':self.reconnectSelectedNodes},{"key":'q','function':self.rearrange},{"key":'shift+x','function':self.disconnectSelectedNodes},{'key':'shift+f','function':self.createFrame}]

    def _getNodeOnLeft(self,nodes):
        tempPos = float('inf')
        leftNode = None
        for node in nodes:
            nodePosX = node.getPosition()[0]
            if nodePosX < tempPos:
                tempPos = nodePosX
                leftNode = node
        return leftNode

    def _getNodeOnRight(self,nodes):
        tempPos = -float('inf')
        rightNode = None
        for node in nodes:
            nodePosX = node.getPosition()[0]
            if nodePosX > tempPos:
                tempPos = nodePosX
                rightNode = node
        return rightNode

    def _getTopNode(self,nodes):
        tempPos = float('inf')
        bottomNode = None
        for node in nodes:
            nodePosY = node.getPosition()[1]
            if nodePosY < tempPos:
                tempPos = nodePosY

                bottomNode = node
        return bottomNode

    def _getBottomNode(self,nodes):
        tempPos = -float('inf')
        topNode = None
        for node in nodes:
            nodePosY = node.getPosition()[1]
            if nodePosY > tempPos:
                tempPos = nodePosY

                topNode = node
        return topNode

    def _getFirstOutputPropertyId(self,node):
        outputProps = node.getProperties(SDPropertyCategory.Output)
        outputProp = None
        if len(outputProps) > 0:
            outputProp = outputProps[0].getId()

        return outputProp
    def _getConnectableInputPropertiesId(self,node):
        if node:
            inputProps = node.getProperties(SDPropertyCategory.Input)
            if len(inputProps) > 0:
                inputConnectableProps = []
                for prop in inputProps:
                    if prop.isConnectable():
                        inputConnectableProps.append(prop.getId())
                if inputConnectableProps == []:
                    inputConnectableProps = None
            else:
                inputConnectableProps = None
            return inputConnectableProps

    
    
    #get two nodes in a pair ,which is connected 
    #output the connection at the same time
    #first is input node ,second is output node ,third is connection
    def _getConnectedNodePairs(self,nodes):
        nodePairs = []
        nodeConsidered = []
        for node in nodes:
            outputProps = node.getProperties(SDPropertyCategory.Output)
            for outputProp in outputProps:
                connections = node.getPropertyConnections(outputProp)
                if len(connections) > 0:
                    for connection in connections:
                        outputNode = connection.getInputPropertyNode()
                        if outputNode not in nodeConsidered:

                            nodePairs.append([node,outputNode,connection])
            nodeConsidered.append(node)

        return nodePairs

    def getDistance(self,a,b):
        ax = a[0]
        ay = a[1]
        bx = b[0]
        by = b[1]
        distance = math.sqrt((ay-by)*(ay-by) + (ax - bx)*(ax - bx))
        return distance
    
    def dot(self,a,b):
        lengthA = self.getDistance(a,[0,0])
        lengthB = self.getDistance(b,[0,0])
        if lengthA != 0:
            normalizedA = [a[0]/lengthA,a[1]/lengthA]
        else:
            normalizedA = a
        if lengthB != 0:
            normalizedB = [b[0]/lengthB,b[1]/lengthB]
        else:
            normalizedB = b
        dot = normalizedA[0]*normalizedB[0] + normalizedA[1]*normalizedB[1]
        return dot
    def _getRelativePropPos(self,outputNode,connect):
        outputProp = connect.getInputProperty()
        outputNodeInputProps = outputNode.getProperties(SDPropertyCategory.Input)
        outputNodeConnectableInputProps = []
        for prop in outputNodeInputProps:
            if prop.isConnectable():
                outputNodeConnectableInputProps.append(prop.getId())
        
        n = len(outputNodeConnectableInputProps)
        i = outputNodeConnectableInputProps.index(outputProp.getId())

        propPos = [0,(i - 0.5 * n + 0.5)*35]

        return propPos


    def _getDotBetweenTargetNodeAndNodePair(self,targetNode,nodePair):
        inputNode = nodePair[0]
        outputNode = nodePair[1]
        connect = nodePair[2]
        
        propPos = self._getRelativePropPos(outputNode,connect)

        inputNodePos = inputNode.getPosition()
        outputNodePos = outputNode.getPosition()
        targetNodePos = targetNode.getPosition()

        v1 = [inputNodePos[0] - targetNodePos[0],inputNodePos[1] - targetNodePos[1]]
        v2 = [outputNodePos[0] - targetNodePos[0],outputNodePos[1] + propPos[1] - targetNodePos[1]]

        dot = self.dot(v1,v2)
        return dot

    def _getCenterPosOfTwoNodes(self,a,b):
        ax = a.getPosition()[0]
        ay = a.getPosition()[1]
        bx = b.getPosition()[0]
        by = b.getPosition()[1]
        centerPos = [(ax + bx)/2,(ay + by)/2]
        return centerPos

    def _getNodePairAcrossTarget(self,targetNode,nodePairs):
        dotDict = {}
        for nodePair in nodePairs:
            dot = self._getDotBetweenTargetNodeAndNodePair(targetNode,nodePair)
            dotDict[dot] = nodePair
        
        acrossPairs = []
        if dotDict != {}:
            sortedDot = sorted(dotDict)
            for dot in sortedDot:
                if dot < -0.9:
                    acrossPairs.append(dotDict[dot])
        if acrossPairs == []:
            acrossPairs = None
        return acrossPairs
    
    def _reconnectNodesConnected(self,targetNode,connectedNodes):
        #split nodes into two parts,one part on the left of target node,one part on the right of target node
        rightNodes = []
        leftNodes = []
        nodePairs = []
        for connectedNode in connectedNodes:
            nodePairs.append([targetNode,connectedNode])
        for nodePair in nodePairs:
            
            rightNode = self._getNodeOnRight(nodePair)
            leftNode = self._getNodeOnLeft(nodePair)
            if rightNode.getIdentifier() != targetNode.getIdentifier():
                rightNodes.append(rightNode)
            if leftNode.getIdentifier() != targetNode.getIdentifier():
                leftNodes.append(leftNode)

        #get orig connection and all the relations of the connected Nodes
        allConnectionRelations = {}
        for leftNode in leftNodes:
            allConnections = []
            
            outputProps = leftNode.getProperties(SDPropertyCategory.Output)
            for outputProp in outputProps:
                connections = leftNode.getPropertyConnections(outputProp)
                for connect in connections:
                    allConnections.append(connect)
            
            for connect in allConnections:
                inputProp = connect.getOutputProperty()
                inputNode = leftNode
                outputProp = connect.getInputProperty()
                outputNode = connect.getInputPropertyNode()

                centerPos = self._getCenterPosOfTwoNodes(inputNode,outputNode)
                propPos = self._getRelativePropPos(outputNode,connect)
                realPropPosY = centerPos[1] + propPos[1]
                

                dot = self._getDotBetweenTargetNodeAndNodePair(targetNode,[inputNode,outputNode,connect])

                if dot < -0.9:
                    allConnectionRelations[realPropPosY] = [connect,inputNode,outputNode,inputProp,outputProp,dot]
        
        #reconnect orig nodes with target node
        sortedCenterPosY = sorted(allConnectionRelations)
        #prepare target node info
        targetInputProps = targetNode.getProperties(SDPropertyCategory.Input)
        targetInputConnectableProps = []
        for prop in targetInputProps:
            if prop.isConnectable():
                targetInputConnectableProps.append(prop)

        targetOutputProps = targetNode.getProperties(SDPropertyCategory.Output)
        targetOutputConnectableProps = []
        for prop in targetOutputProps:
            if prop.isConnectable():
                targetOutputConnectableProps.append(prop)
        
        
        #input node reconnect
        for i in range(len(sortedCenterPosY)):
            #prepare nodes info
            inputNode = allConnectionRelations[sortedCenterPosY[i]][1]
            outputNode = allConnectionRelations[sortedCenterPosY[i]][2]
            inputProp = allConnectionRelations[sortedCenterPosY[i]][3]
            outputProp = allConnectionRelations[sortedCenterPosY[i]][4] 
            dot = allConnectionRelations[sortedCenterPosY[i]][5]

            if i < len(targetInputConnectableProps):
                #when there is only one connection and the target node is blend node, connect to the blend node's destination prop
                if len(sortedCenterPosY) == 1 and targetNode.getDefinition().getId() == 'sbs::compositing::blend':
                    inputNode.newPropertyConnection(inputProp,targetNode,targetInputConnectableProps[1])
                else:
                    inputNode.newPropertyConnection(inputProp,targetNode,targetInputConnectableProps[i])
            
                #output node reconnect
                if len(targetOutputConnectableProps) > 0:
                    targetNode.newPropertyConnection(targetOutputConnectableProps[0],outputNode,outputProp)
                #self._moveNode(inputNode,[100,0])
                #self._moveNode(outputNode,[100,0])

        
                

    def _moveNode(self,node,vector):
        origPos = node.getPosition()
        node.setPosition(float2(origPos[0] + vector[0],origPos[1] + vector[1]))

    #connect multi nodes to another node
    def connectNodes(self,inputNodes,outputNode):
        # if input node only one
        if len(inputNodes) == 1:
            inputNode = inputNodes[0]

            inputNodeOutputProp = self._getFirstOutputPropertyId(inputNode)
            outputNodeInputProps = self._getConnectableInputPropertiesId(outputNode)
    
            if outputNodeInputProps is not None:
                if inputNodeOutputProp is not None:  
                    #only comp node connect to the destination prop
                    if outputNode.getDefinition().getId() == 'sbs::compositing::blend':
                        index = 1
                    else:
                        index = 0
                    inputNode.newPropertyConnectionFromId(inputNodeOutputProp,outputNode,outputNodeInputProps[index])
       
        #if input nodes is multi
        elif len(inputNodes) > 1:
            firstTwoNodes = [inputNodes[0],inputNodes[1]]
            bottomNode = self._getBottomNode(firstTwoNodes)
            topNode = self._getTopNode(firstTwoNodes)

            #some time the two nodes are even, tell them by x position
            if topNode == bottomNode:
                topNode = self._getNodeOnLeft(firstTwoNodes)
                bottomNode = self._getNodeOnRight(firstTwoNodes)
            
            bottomNodeOutputProp = self._getFirstOutputPropertyId(bottomNode)
            topNodeOutputProp = self._getFirstOutputPropertyId(topNode)

            outputNodeInputProps = self._getConnectableInputPropertiesId(outputNode)

            if outputNodeInputProps is not None:
                if bottomNodeOutputProp is not None:
                    try:
                        bottomNode.newPropertyConnectionFromId(bottomNodeOutputProp,outputNode,outputNodeInputProps[1])
                    except:
                        bottomNode.newPropertyConnectionFromId(bottomNodeOutputProp,outputNode,outputNodeInputProps[0])
                if topNodeOutputProp is not None:
                    try:
                        topNode.newPropertyConnectionFromId(topNodeOutputProp,outputNode,outputNodeInputProps[0])
                    except:
                        pass

    def reconnectSelectedNodes(self):
        def _getDistanceBetweenNodes(a,b):
            aPos = a.getPosition()
            bPos = b.getPosition()
            xD = bPos[0] - aPos[0]
            yD = bPos[1] - aPos[1]
            d = math.sqrt(xD*xD + yD*yD)
            return d

        def _getClosetNodes(sel,others):
            closetNodesNumber = 4
            currentPos = sel.getPosition()
            distance_nodeDict = {}
            closetNodes = []
            for other in others:
                selId = sel.getIdentifier()
                otherId = other.getIdentifier()
                if selId != otherId:
                    distance = _getDistanceBetweenNodes(sel,other)
                    
                    distance_nodeDict[distance] = other
            sortedDistance = sorted(distance_nodeDict)
            for i in range(closetNodesNumber):
                closetNodes.append(distance_nodeDict[sortedDistance[i]])
            return closetNodes
           

        self.sels = self.UIMgr.getCurrentGraphSelection()
        if len(self.sels) > 1:
            rightNode = self._getNodeOnRight(self.sels)
            leftNodes = []
            for sel in self.sels:

                if sel.getIdentifier() != rightNode.getIdentifier():
                    
                    leftNodes.append(sel)

            self.connectNodes(leftNodes,rightNode)
        if len(self.sels) == 1:
            sel = self.sels[0]
            self.currentGraph = self.UIMgr.getCurrentGraph()
            allNodes = self.currentGraph.getNodes()
            closetNodes = _getClosetNodes(sel,allNodes)
 
            connectedPairs = self._getConnectedNodePairs(closetNodes)
            acrossPairs = self._getNodePairAcrossTarget(sel,connectedPairs)

            #split pairs node into one group, and delete the same node
            if acrossPairs:
                acrossNodes = []
                for acrossPair in acrossPairs:
                    acrossNodes.append(acrossPair[0])
                    acrossNodes.append(acrossPair[1])
                for fnode in acrossNodes:
                    for snode in acrossNodes:
                        if fnode != snode:
                            if fnode.getIdentifier() == snode.getIdentifier():
                                acrossNodes.remove(fnode)       
                self._reconnectNodesConnected(sel,acrossNodes)
    
    def disconnectSelectedNodes(self):
        print ('disconnect')
        self.sels = self.UIMgr.getCurrentGraphSelection()

        

        if self.sels:
            for sel in self.sels:
                inputProp_inputNode = {}
                outputProp_outputNode = {}
                inputProps = []
                outputProps = []
                #reconnect other nodes, connected to the selection before
                selInputProps = sel.getProperties(SDPropertyCategory.Input)
                selOutputProps = sel.getProperties(SDPropertyCategory.Output)
                for inputProp in selInputProps:
                    connections = sel.getPropertyConnections(inputProp)
                    for connect in connections:
                        prop = connect.getInputProperty()
                        inputNode = connect.getInputPropertyNode()
                        inputProp_inputNode[prop] = inputNode
                        inputProps.append(prop)
                for outputProp in selOutputProps:
                    connections = sel.getPropertyConnections(outputProp)
                    for connect in connections:
                        prop = connect.getInputProperty()
                        outputNode = connect.getInputPropertyNode()
                        outputProp_outputNode[prop] = outputNode
                        outputProps.append(prop)
                inputN = len(inputProps)
                outputN = len(outputProps)
                if inputN > 0 and outputN > 0:
                    for i in range(inputN):
                        if i <= outputN - 1:
                            k = i
                        else:
                            k = min(i,outputN - 1)

                        inputProp = inputProps[i]
                        inputNode = inputProp_inputNode[inputProps[i]]
                        outputProp = outputProps[k]
                        outputNode = outputProp_outputNode[outputProps[k]]

                        inputNode.newPropertyConnection(inputProp,outputNode,outputProp)
                
                #delete selected node all connections
                inputProps = sel.getProperties(SDPropertyCategory.Input)
                outputProps = sel.getProperties(SDPropertyCategory.Output)
                props = []
                for prop in inputProps:
                    if prop.isConnectable():
                        props.append(prop)
                for prop in outputProps:
                    if prop.isConnectable():
                        props.append(prop)
                for prop in props:
                    sel.deletePropertyConnections(prop)

    def _moveNodeByAngleDistance(self,posA,posB):
        dx = posB[0] - posA[0]
        dy = posB[1] - posA[1]
        offset = [0,0]
        dot1 = self.dot([dx,dy],[1,0])
        dot2 = self.dot([dx,dy],[0,1])
        #when two nodes is close, move B out of the room

        if dx < self.roomX and math.fabs(dy) < self.roomY:
            
            offsetX = self.roomX - dx
            offsetY = self.roomY - math.fabs(dy)
            if dy < 0:
                offsetY = -offsetY
            
            if dot1 > 0.707:
                offset = [offsetX,0] 
            else:
                offset = [0,offsetY]
        # when two nodes are close even, make them even
        if dot1 > self.snapAngle:
            offset = [offset[0],offset[1] - dy]
        if dot2 > self.snapAngle or dot2 < -self.snapAngle:
            offset = [offset[0] - dx,offset[1]]

        return offset

    #if the node have only one input, ignor
    #if the node have multi inputs, adjust the node's pos,which hooked into the firstprop
    def _moveNodeByConnections(self):
        self.self = self.UIMgr.getCurrentGraphSelection()
        for node in self.self:

            pos = node.getPosition()
            inputProps = node.getProperties(SDPropertyCategory.Input)
            inputConnectableProps = [] 
            for prop in inputProps:
                if prop.isConnectable():
                    inputConnectableProps.append(prop)
            
            n = len(inputConnectableProps)
            for i in range(n):
                #this is the target offset of the input node 
                #consider the prop position 
                #only first input node need to adjust
                #else aligned to the base node 
                if i == 0 :
                    targetOffsetY = (i - 0.5*n + 0.5)*self.roomY*1.6 + pos[1]
                else:
                    targetOffsetY = pos[1]
                connections = node.getPropertyConnections(inputConnectableProps[i])
            
                for connect in connections:
                    inputNode = connect.getInputPropertyNode()


                    inputNodePos = inputNode.getPosition()

                    inputProp = connect.getInputProperty()
                    
                    inputPropConnections = inputNode.getPropertyConnections(inputProp)
                    
                    inputNodeOutputNodes = []
                    centerPos = [0,0]
                    
                    if len(inputPropConnections) > 1:
                        for inputPropConnection in inputPropConnections:
                            tempNode = inputPropConnection.getInputPropertyNode()
                            for sel in self.sels:
                                if sel.getIdentifier() == tempNode.getIdentifier():
                                    inputNodeOutputNodes.append(tempNode)

                        
                        posSum = [0,0]
                        m = len(inputNodeOutputNodes)
                        if m >1:
                            for tempNode in inputNodeOutputNodes:
                                posSum = [posSum[0] + tempNode.getPosition()[0],posSum[1] + tempNode.getPosition()[1]]
                            centerPos = [posSum[0]/m,posSum[1]/m]
                            targetOffsetY = centerPos[1]
                    


                    dx = pos[0] - inputNodePos[0]
                    offsetX = 0
                    if dx < self.roomX:
                        offsetX = -(self.roomX - dx)
                    
                    offsetY = targetOffsetY - inputNodePos[1]
                    
                    for sel in self.sels:
                        if sel.getIdentifier() ==inputNode.getIdentifier():
                            self._moveNode(inputNode,[offsetX,offsetY*1])


    def rearrange(self):
        self.roomX = 150
        self.roomY = 130
        self.snapAngle = 0.98


        self.currentGraph = self.UIMgr.getCurrentGraph()
        self.sels = self.UIMgr.getCurrentGraphSelection()
        allNodes = self.currentGraph.getNodes()
        
        initFirstNode = False
        nodeOffsetRelation = {}
        startNode = None
        bottomNode = None
        leftNode = None
        minPosX = float('inf')
        minPosY = -float('inf')
        offset = [0,0]
        
        if len(self.sels)>1:
            for fnode in self.sels:
                fpos = fnode.getPosition()
                dis = float('inf')
                if fpos[1] > minPosY:
                    minPosY = fpos[1]
                    bottomNode = fnode
                if fpos[0] < minPosX:
                    minPosX = fpos[0]
                    leftNode = fnode


                nearestNode = None
                for snode in self.sels:
                    if snode.getIdentifier() != fnode.getIdentifier():
                        spos = snode.getPosition()
                        if spos[0] > fpos[0]:
                            tempDis = self.getDistance(fpos,spos)
                            if tempDis <= dis:
                                dis = tempDis
                                nearestNode = snode
                        #when two nodes are aligned v dir, give them a pointer;infinite loop otherwise
                        if spos[0] == fpos[0] and spos[1] > fpos[1]:
                            tempDis = self.getDistance(fpos,spos)
                            if tempDis <= dis:
                                dis = tempDis
                                nearestNode = snode

                if nearestNode:
                    nearestPos = nearestNode.getPosition()

                    offset = self._moveNodeByAngleDistance(fpos,nearestPos)

                nodeOffsetRelation[fnode.getIdentifier()] = [fnode,nearestNode,offset[0],offset[1]]

            
            bottomToLeftVector = [leftNode.getPosition()[0] - bottomNode.getPosition()[0],leftNode.getPosition()[1] - bottomNode.getPosition()[1]]
            
            #get left bottom corner node as start node
            if self.dot(bottomToLeftVector,[1,-1]) >0:
                startNode = bottomNode
            else:
                startNode = leftNode

            nodeOffsetAddup = nodeOffsetRelation


            #add all the previous nodes' offset up
            def _offsetAdd(thisNode):
                
                thisNodeId = thisNode.getIdentifier()
                
                nextNode = nodeOffsetAddup[thisNodeId][1]
                if nextNode:
                    nextNodeId = nextNode.getIdentifier()
                    nextNodeOffset = [nodeOffsetAddup[thisNodeId][2],nodeOffsetAddup[thisNodeId][3]]

                    if nextNodeId in nodeOffsetAddup:
                        nextAfterNode = nodeOffsetAddup[nextNodeId][1]
                        if nextAfterNode:
                            nextAfterNodeId = nextAfterNode.getIdentifier()
                            nextAfterNodeOffset = [nodeOffsetAddup[nextNodeId][2],nodeOffsetAddup[nextNodeId][3]]

                            #keep faraway nodes from moving
                            np = [nextNode.getPosition()[0],nextNode.getPosition()[1]]
                            ap = [nextAfterNode.getPosition()[0],nextAfterNode.getPosition()[1]]
                            dx = ap[0] - np[0]
                            dy = ap[1] -np[1]

                            # the next node is very far away
                            if dx > self.roomX or math.fabs(dy) > self.roomY:
                                nnp = [nextNode.getPosition()[0] + nextNodeOffset[0],nextNode.getPosition()[1] + nextNodeOffset[1]]
                                ap = [nextAfterNode.getPosition()[0],nextAfterNode.getPosition()[1]]
                                nextAfterNodeRealOffset = self._moveNodeByAngleDistance(nnp,ap)
                                dx = ap[0] - nnp[0]
                                dy = ap[1] - nnp[1]
                                if dx < self.roomX and math.fabs(dy) < self.roomY:
                                    
                                    nodeOffsetAddup[nextNodeId] = [nextNode,nextAfterNode,nextAfterNodeRealOffset[0],nextAfterNodeRealOffset[1]]
                                else:
                                    nextAfterNodeRealOffset = [0,0]
                                    nodeOffsetAddup[nextNodeId] = [nextNode,nextAfterNode,nextAfterNodeRealOffset[0],nextAfterNodeRealOffset[1]]
                            #the next node is in range
                            else:
                                nextAfterNodeRealOffset = [nextNodeOffset[0] + nextAfterNodeOffset[0],nextNodeOffset[1] + nextAfterNodeOffset[1]]
                                nodeOffsetAddup[nextNodeId] = [nextNode,nextAfterNode,nextAfterNodeRealOffset[0],nextAfterNodeRealOffset[1]]
                                    
                            #nextAfterNodeRealOffset = [nextNodeOffset[0] + nextAfterNodeOffset[0],nextNodeOffset[1] + nextAfterNodeOffset[1]]
                            #nodeOffsetAddup[nextNodeId] = [nextNode,nextAfterNode,nextAfterNodeRealOffset[0],nextAfterNodeRealOffset[1]]

                            if nextAfterNodeId not in nodeOffsetAddup:
                                return
                            return _offsetAdd(nextNode)

            _offsetAdd(startNode)



            for nodeId in nodeOffsetAddup:
                moveNode = nodeOffsetAddup[nodeId][1]

                offset = [nodeOffsetAddup[nodeId][2],nodeOffsetAddup[nodeId][3]]
                if moveNode:
                    self._moveNode(moveNode,offset)
            

            sortedNotes = [startNode]
            def _sortNodes(node):
                if node is not None and node.getIdentifier() in nodeOffsetRelation:
                    nextNode = nodeOffsetRelation[node.getIdentifier()][1]
                    sortedNotes.append(node)
                    return _sortNodes(nextNode)
                else:
                    return sortedNotes

            _sortNodes(startNode)

            self._moveNodeByConnections()

            print ('rearrange')

    def createFrame(self):
        self.sels = self.UIMgr.getCurrentGraphSelection()
        expand = [100,100]
        sizeX = None
        sizeY = None
        leftNode = None
        select = False
        if self.sels:
            
            self.currentGraph = self.UIMgr.getCurrentGraph()
            leftNode = self._getNodeOnLeft(self.sels)
            rightNode = self._getNodeOnRight(self.sels)
            topNode = self._getTopNode(self.sels)
            bottomNode = self._getBottomNode(self.sels)

            leftPos = leftNode.getPosition()
            rightPos = rightNode.getPosition()
            topPos = topNode.getPosition()
            bottomPos = bottomNode.getPosition()

            sizeX = rightPos[0] - leftPos[0] + 2*expand[0]
            sizeY = bottomPos[1] - topPos[1] + 2*expand[1]
            select = True
        frame = SDGraphObjectFrame.sNew(self.currentGraph)

        if select:
            frame.setSize(float2(sizeX,sizeY))
        if select:
            frame.setPosition(float2(leftPos[0] - expand[0],topPos[1] - expand[1]))

class NodeCreator(ShortcutsFunctions):
    def __init__(self,item):
        super(NodeCreator,self).__init__()
        #global vars, could be modified
        #the offset of the node new created
        self.rightOffset = 150

        #get base infomations
        self.nodeName = None
        self.graphUrl = None
        self.currentGraph = self.UIMgr.getCurrentGraph()
        self.currentGraphType =  self.currentGraph.getClassName()
        if self.currentGraphType in item:
            self.nodeName = item[self.currentGraphType]['node']
            

            self.inputProps = []
            self.inputs = []
            self.inputPropertyInheritanceMethods = []
            if 'props' in item[self.currentGraphType]:
                self.inputProps = item[self.currentGraphType]['props']
            if 'inputs' in item[self.currentGraphType]:
                self.inputs = item[self.currentGraphType]['inputs']
            if 'propertyInheritanceMethods' in item[self.currentGraphType]:
                self.inputPropertyInheritanceMethods = item[self.currentGraphType]['propertyInheritanceMethods']
            if 'graphUrl' in item[self.currentGraphType]:
                self.graphUrl = item[self.currentGraphType]['graphUrl']

        self.newNode = None

    def _createInstanceNode(self):
        self.currentGraph = self.UIMgr.getCurrentGraph()
        self.currentGraph.sNewFromFile
        self.pkgMgr = self.app.getPackageMgr()
        resourcePath =self.app.getPath(SDApplicationPath.DefaultResourcesDir)
        instanceNode = None
        if self.nodeName:
            package = self.pkgMgr.loadUserPackage(os.path.join(resourcePath,'packages',self.nodeName))
            if package:
                packageName = ''
                if self.graphUrl:
                    packageName = self.graphUrl
                else:
                    packageName = self.nodeName.split('.')[0]
                instanceNode = self.currentGraph.newInstanceNode(package.findResourceFromUrl(packageName))
            else:
                print ('no such package ,maybe name error')
        #this may cause lag ,need more test
        #self.pkgMgr.unloadUserPackage(package)
        return instanceNode
    
    def createNewNode(self):
        self.currentGraph = self.UIMgr.getCurrentGraph()
        if self.currentGraph and self.nodeName:
            if '::' in self.nodeName:
                newNode = self.currentGraph.newNode(self.nodeName)
            elif 'sbs' in self.nodeName:
                newNode = self._createInstanceNode()
            self.newNode = newNode

            if len(self.inputPropertyInheritanceMethods) > 0:
                method = None
                for inheritanceMethod in self.inputPropertyInheritanceMethods:
                    print (inheritanceMethod)
                    if inheritanceMethod['value'] == 'Absolute':
                        method = SDPropertyInheritanceMethod.Absolute
                    elif inheritanceMethod['value'] == 'RelativeToInput':
                        method = SDPropertyInheritanceMethod.RelativeToInput
                    elif inheritanceMethod['value'] == 'RelativeToParent':
                        method = SDPropertyInheritanceMethod.RelativeToParent
                    newNode.setInputPropertyInheritanceMethodFromId(inheritanceMethod['id'],method)

            if len(self.inputProps) > 0:
                for prop in self.inputProps:
                    value = None
                    if prop["type"] == 'int':
                        value = SDValueInt.sNew(prop["value"])
                    elif prop["type"] == 'int2':
                        value = SDValueInt2.sNew(int2(prop["value"][0], prop["value"][1]))
                    elif prop["type"] == 'int3':
                        value = SDValueInt3.sNew(int3(prop["value"][0], prop["value"][1], prop["value"][2]))
                    elif prop["type"] == 'int4':
                        value = SDValueInt4.sNew(int4(prop["value"][0], prop["value"][1], prop["value"][2], prop["value"][3]))
                    elif prop["type"] == 'float':
                        value = SDValueFloat.sNew(prop["value"])
                    elif prop["type"] == 'float2':
                        value = SDValueFloat2.sNew(float2(prop["value"][0], prop["value"][1]))
                    elif prop["type"] == 'float3':
                        value = SDValueFloat3.sNew(float3(prop["value"][0], prop["value"][1], prop["value"][2]))
                    elif prop["type"] == 'float4':
                        value = SDValueFloat4.sNew(float4(prop["value"][0], prop["value"][1], prop["value"][2], prop["value"][3]))
                    elif prop["type"] == 'bool':
                        value = SDValueBool.sNew(prop["value"])
                    elif prop["type"] == 'enum':
                        value = SDValueInt.sNew(prop["value"])
                    elif prop["type"] == 'string':
                        value = SDValueString.sNew(prop["value"])
                    newNode.setInputPropertyValueFromId(prop["id"], value)
            
            


    def _setNewNodeToMousePos(self):
        pos = QtGui.QCursor.pos()
        qtGraphWidget = self.mainWindow.childAt(pos)

        if qtGraphWidget:
            qtGraphViewer = qtGraphWidget.parent()
            if qtGraphViewer.metaObject().className() == 'Pfx::Editor::Components::Graph::GraphView':
                
                viewPos = qtGraphViewer.mapFromGlobal(QtGui.QCursor().pos())
                scenePos = qtGraphViewer.mapToScene(viewPos)

                moveTo = float2(scenePos.x(),scenePos.y())
                self.newNode.setPosition(moveTo)

    
    def setNewNodePosition(self):
        self.sels = self.UIMgr.getCurrentGraphSelection()
        if self.newNode:
            if len(self.sels) < 1:
                self._setNewNodeToMousePos()

            elif len(self.sels) == 1:
                sel = self.sels[0]
                selNodePos = sel.getPosition()
                if self.newNode:
                    self.newNode.setPosition(float2(selNodePos[0] + self.rightOffset,selNodePos[1]))

            elif len(self.sels) >1:
                firstTwoNodes = [self.sels[0],self.sels[1]]
                rightNode = self._getNodeOnRight(firstTwoNodes)
                posX = rightNode.getPosition()[0] + self.rightOffset
                
                yTotal = 0
                for node in firstTwoNodes:
                    yTotal += node.getPosition()[1]
                posY = yTotal/len(firstTwoNodes)
                self.newNode.setPosition(float2(posX,posY))

    def connectSelectedNodesToNewNode(self):
        self.connectNodes(self.sels,self.newNode)




def readShortcutFile():
    __currentPath__ = os.path.dirname(__file__)
    configFilePath = os.path.join(__currentPath__,r'config\shortcuts.json')

    with open(configFilePath, 'r') as f:
        shortcuts = json.load(f)
    return (shortcuts)

def excuteShotcutsCreateNewNode(item):
    nodeCreator = NodeCreator(item)
    nodeCreator.createNewNode()
    nodeCreator.setNewNodePosition()
    nodeCreator.connectSelectedNodesToNewNode()



def excuteSpecialFunction(key):
    shortcutsFunctions = ShortcutsFunctions()
    funcList = shortcutsFunctions.functions 

    for funcDict in funcList:
        if key in funcDict.values():

            funcDict['function']()



def createMenu():
    UIMgr = getUIMgr()
    UIMgr.deleteMenu('')
    menu = UIMgr.newMenu('','shortcutMenu')

    shortcuts = readShortcutFile()

    for item in shortcuts:
        action = menu.addAction('new')
        action.setShortcut(QtGui.QKeySequence(item['key']))
        action.triggered.connect(lambda f = excuteShotcutsCreateNewNode,arg = item:f(arg))
   
    shortcutsFunctions = ShortcutsFunctions()
    for item in shortcutsFunctions.functions:
        action = menu.addAction('new')
        action.setShortcut(QtGui.QKeySequence(item['key']))
        action.triggered.connect(lambda f = excuteSpecialFunction, arg = item['key']:f(arg))




