#!/usr/bin/env python

__author__ = "Jenna Reher"
__credits__ = "Eric Cousineau"
__email__ = "jreher@caltech.edu"

import sys
import datetime
import re

def addone(obj):
    val = int(obj.group(1))
    return str(val+1)

def ExtractOutSize(fname):
    hhName = fname + '.hh'
    with open(hhName) as fp:
        sizeHeader = 'assert_size_matrix\(p_output1, '
        outSize = re.findall(sizeHeader + '(.*?)\);', fp.read(), re.S)[0]
        return outSize
        
def ExtractInSize(fname):
    hhName = fname + '.hh'
    with open(hhName) as fp:
        inAssertHead = 'assert_size_matrix\(var'
        inBlocks = re.findall(inAssertHead + '(.*?)', fp.read(), re.S)
        return len(inBlocks)

def ExtractMainFunctionBlock(fname, inSize):
    ccName = fname + '.cc'
    print(ccName + '-> .m')
    with open(ccName) as fp:
        functionHeader = 'void output1\(double \*p_output1'
        for index in range(inSize):
            functionHeader = functionHeader + ',const double \*var' + str(index+1)
        
        functionHeader = functionHeader + '\)\n{'            
        mainFunctionBlock = re.findall(functionHeader + '(.*?)}', fp.read(), re.S)[0]
        return mainFunctionBlock    
         
def ParseBlockIntoMatlab(fname, functionBlock, outSize, inSize):
    matFile = open(fname + '.m', 'w')
    name = re.sub('^(.*\\\\)', '', fname)
    name = re.sub('^(.*/)', '', name)
    
    # Write a comment header
    timestamp = datetime.datetime.now().strftime("%B %d, %Y T %I:%M")
    commentHead = '%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n'
    commentHead = commentHead + '% ' + name + '\n% \n'
    commentHead = commentHead + '% This file is automatically converted to matlab format from C code.\n'
    commentHead = commentHead + '% Source symbolic expressions exported from FROST:\n'
    commentHead = commentHead + '% \t https://github.com/ayonga/frost-dev\n'
    commentHead = commentHead + '% \n% Author: Jenna Reher (jreher@caltech.edu)\n'
    commentHead = commentHead + '% Automatically generated on: ' + timestamp + '\n'
    commentHead = commentHead + '%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n'
    
    matFile.write(commentHead)
    
    # Remove double and NULL declarations
    functionBlock = re.sub(r'  double.*\n?','', functionBlock)
    functionBlock = re.sub(r'  NULL.*\n?','', functionBlock)
    
    # Replace common functions
    functionBlock = re.sub(r'Abs', 'abs', functionBlock)
    functionBlock = re.sub(r'Sqrt', 'sqrt', functionBlock)
    functionBlock = re.sub(r'ArcCos', 'acos', functionBlock)
    functionBlock = re.sub(r'ArcSin', 'asin', functionBlock)
    functionBlock = re.sub(r'Cos', 'cos', functionBlock)
    functionBlock = re.sub(r'Sin', 'sin', functionBlock)
    functionBlock = re.sub(r'Exp', 'exp', functionBlock)
    functionBlock = re.sub(r'Log', 'log', functionBlock)
    functionBlock = re.sub(r'Power', 'power', functionBlock)
    functionBlock = re.sub(r'Tan', 'tan', functionBlock)
    
    # Create parseblock and reassign line by line to new block
    editedBlock = ''
    parseBlock = functionBlock.splitlines()
    
    for line in parseBlock:
        # Increment all indexing for all vars
        for index in range(inSize):
            curVar = 'var' + str(index+1)
            line = re.sub(r'var{0}\[(\d+)\]'.format(int(index+1)), lambda m: 'var{0}[{1}]'.format(int(index+1), int(m.group(1))+1), line)

        # Increment all indexing for p_output1
        outIndices = re.findall('p_output1\[(.*?)\]', line, re.S)
        for var in reversed(outIndices):
            line = re.sub(r'p_output1\[' + var + '\]', 'p_output1[' + str(int(var) + 1) + ']', line)
        
        if ']=0;' not in line:
        	editedBlock = '\n'.join((editedBlock, line))
        
    # Convert bracket indexing to matlab
    editedBlock = re.sub(r'\[', '(',  editedBlock)
    editedBlock = re.sub(r'\]', ')',  editedBlock)
    
    # Get size for reshaping
    outSizeSplit = outSize.split(',')
    m = outSizeSplit[0]
    n = outSizeSplit[1]
    fullSize = int(n) * int(m)
    
    # Write the function header, block, and endfile
    matlabHeader = 'function out = ' + name + '(var1'
    for index in range(inSize-1):
        matlabHeader = matlabHeader + ',var' + str(index+2)
    matlabHeader = matlabHeader + ')\n'
    
    matFile.write(matlabHeader)
    matFile.write('\n  out = zeros(' + outSize + ');\n')
    matFile.write('\n  p_output1 = zeros(' + str(fullSize) + ',1);\n')
        
    matFile.write(editedBlock)
    
    # Reshape the vector into the desired output
    matFile.write('\n  out = reshape(p_output1, ' + m + ',' + n + ');\n')
    
    # Closes
    matFile.write('end')
    
    # Close down
    matFile.close()
         
    
if __name__ == '__main__':
    src_name = sys.argv[1]
    dest_name = sys.argv[2]
    outSize = ExtractOutSize(src_name)
    inSize = ExtractInSize(src_name)
    mainFunc = ExtractMainFunctionBlock(src_name, inSize)
    ParseBlockIntoMatlab(dest_name, mainFunc, outSize, inSize)