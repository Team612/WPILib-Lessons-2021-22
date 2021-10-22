import os
import shutil
import random

files = os.listdir('./susImages') #saves files
numfiles = len(os.listdir('./susImages')) #print out number of files in directory
print(numfiles)
while numfiles < (6120 - 2):
    for fname in files: #iterate through files
        newfname = fname.replace('.png', ' copy.png') #add new filename to prevent errors
        path = os.path.join('./susImages', fname) #join paths
        newpath = os.path.join('./susImages', newfname)
        # copying the files to the
        # destination directory
        shutil.copy2(path, newpath) #copy directory
        numfiles = len(os.listdir('./susImages')) #reset numfiles var
        if numfiles % 100 == 0:
            print(numfiles)

files = os.listdir('./susImages') #saves files again

intervals = [10000, 1000, 100, 10, 1]
for i in intervals:
    while len(files) - (6120 -2) >= i:
        filesToBeRemoved = random.choices(files, k = i)
        for file in filesToBeRemoved:
            os.remove(os.path.join('./susImages', file))
        files = os.listdir('./susImages')