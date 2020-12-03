from pathlib import Path
from argparse import ArgumentParser
import json
import os
import copy
import re
class CodeTemplate:
    def __init__(self, code):
        self.code = code
        self.args = {}
        self.indent = ''
    def setArgs(self, args):
        self.args = args
    def _replace(self, m):
        if m.group(2) in self.args:
            return m.group(1) + self.args[m.group(2)].replace('\n','\n'+m.group(1))
        raise Exception('No replacement specified for argument '+m.group(1))
    def generate(self):
        data = copy.deepcopy(self.code)
        res= re.sub('(\t*)\${([a-zA-Z]+)}', self._replace, data)
        if self.indent:
            res = re.sub('^(.)',r'{}\1'.format(self.indent),res,flags=re.MULTILINE)
        return res

if __name__ == '__main__':
    parser = ArgumentParser(description='Generates the message scripts file with all implementations')
    parser.add_argument('dataFile',help='The data file containing the messages', type=str)
    parser.add_argument('messageTemplate',help='The message template to use',type=str)
    parser.add_argument('targetOutputFile',help='The output file to write to',type=str)
    parser.add_argument('-p','--prefixFile',help='File with content to place before generated code')
    parser.add_argument('-e','--postfixFile',help='File with content to place after generated code')
    parser.add_argument('-i','--indent',type=int, default=1, help='Number of tabs to indent the message classes')
    parser.add_argument('-a','--generateAllMessages',action='store_true',  help='Generate an AllMessages tuple with all message types.')

    currP = Path(os.getcwd())
    args = parser.parse_args()
    data = None
    with open(Path(os.getcwd()) / args.dataFile) as f:
        data = json.load(f)
    templateStr = None
    with open(Path(os.getcwd()) / args.messageTemplate) as f:
        templateStr = f.read()
    template = CodeTemplate(templateStr)
    template.indent = '\t'*args.indent
    with open(Path(os.getcwd()) / args.targetOutputFile, 'w') as f:
        if args.prefixFile:
            with open(currP / args.prefixFile) as pre:
                f.write(pre.read())
        for m in data['messages']:
            replaceData = {}
            replaceData['clsName'] = m['clsName']
            # Create member definitions
            replaceData['memberDefs'] = '\n'.join([
                    '{} m_{};'.format(member['type'], member['name']) for member in m['parameters'] 
                ])
            replaceData['memberAccessors'] = '\n'.join([
                    '{0} {1}() const {{ return m_{1};}}\nvoid set{2}(const {0}& {1}){{m_{1} = {1};}}'.format(member['type'], member['name'],member['name'].capitalize()) 
                    for member in m['parameters'] 
                ])
            replaceData['typeNameQuoted'] = '"' + m['type'] + '"'
            replaceData['encodeCode'] = 'buff << ' + ' << '.join(['m_{}'.format(member['name']) for member in m['parameters']]) +';'
            replaceData['decodeCode'] = 'buff >> ' + ' >> '.join(['m_{}'.format(member['name']) for member in m['parameters']]) + ';'
            template.setArgs(replaceData)
            f.write(template.generate())
            f.write('\n')
        
        if args.generateAllMessages:
            f.write(
                '{}using AllMessages = std::tuple<{}>;\n'.format(template.indent, ','.join([m['clsName'] for m in data['messages']]))
            )
        
        if args.postfixFile:
            with open(currP / args.postfixFile) as post:
                f.write(post.read())

