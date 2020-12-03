import re

class StringIterator:
    def __init__(self, strVal):
        self.strVal = strVal
        # Index of character to be read
        self.index = 0
        self.mark = -1
    # Make iterable
    def __iter__(self):
        return self
    def __next__(self):
        if self.atEnd():
            raise StopIteration
        x = self.curr()
        self.index += 1

        return x
    def __len__(self):
        return len(self.strVal)
    def get(self, index):
        return self.strVal[index]
    def mark(self):
        self.mark = self.index
    def unmark(self):
        self.mark = -1
    def substringFromMark(self):
        return self.strVal[self.mark:self.index]
    def curr(self):
        return self.strVal[self.index]
    def moveNext(self, amount=1):
        self.index += amount
    def nextEquals(self, strVal):
        if len(strVal) + self.index > len(self.strval):
            return False
        for i in range(len(strVal)):
            if self.get(self.index + i) != strVal[i]:
                return False
        return True
    def getNext(self):
        return self.strVal[self.index+1]
    def atEnd(self):
        return self.index >= len(self.strVal)
    
class PlainText:
    def __init__(self, text):
        self.text = text
    def write(self, writable, scope):
        writable.write(self.text)
class Literal:
    def __init__(self):
        self._key = None
        self._modifier = None
    def parse(self, si):
        si.mark()
        while not si.atEnd():
            if si.curr() == '}':
                break
        text = si.substringFromMark()
        parts=  text.split('|')
        self._key = parts[0].strip()
        if len(parts) > 1:
            self._modifier = parts[1]
    def write(self, writable, scope):
        pass
class ForeachConstruct:
    def __init__(self, loopVarName, loopExpression):
        self._loopVar = loopVarName
        self._loopExpression = loopExpression
        self._subTree=  []
    def _parse(self, si, parentParser):
        si.moveNext(len('foreach'))
        si.mark()
        while not si.atEnd():
            if si.curr() == '}':
                break
        self._loopExpression = si.substringFromMark()
        if len(self._loopExpression) == 0:
            raise Exception("Empty foreach construct")
        
    def write(self, writable):
        pass

class Token:
    def __init__(self, tokenType, start,end):
        self.type = tokenType
        self.start = start
        self.end = end
        self.associatedEnd = None
        
    def setTokenAssociatedEnd(self, end):
        self.associatedEnd = end

class TemplateParser:
    def __init__(self, codeTemplate):
        self._template = codeTemplate
        self._parseTemplate()
        self._execTree = None
        self._parseStack = []
        self._tokens = []
    def _parseConstruct(self, si, parseStack):
        si.moveNext(2)
        if si.nextEquals('foreach'):
            pass
        elif si.nextEquals('if'):
            pass
        return False
    def _parseLiteral(self, si, parseStack):
        si.moveNext(2)

        return False

    def _parseMore(self, targetParseStack, si, constructCb, literalCb):
        si.mark()
        while not si.atEnd():
            if si.curr() == '@' and si.getNext() == '{':
                subData = si.substringFromMark()
                if len(subData) > 0:
                    targetParseStack.append(PlainText(subData))
                if constructCb(si,targetParseStack):
                    break
            elif si.curr() == '$' and si.getNext() == '{':
                subData = si.substringFromMark()
                if len(subData) > 0:
                    targetParseStack.append(PlainText(subData))
                if literalCb(si,targetParseStack):
                    break

    def _parseTemplate(self):
        si = StringIterator(self._template)
        self._parseMore(self._parseStack, si, self._parseConstruct, self._parseLiteral)

