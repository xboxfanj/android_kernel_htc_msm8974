#!/usr/bin/python
# This script calculates the HMAC-SHA256 of one file and writes the result
# to stdout

import binascii
import hashlib
import hmac
import sys

from elftools.elf.constants import SH_FLAGS
from elftools.elf.descriptions import _DESCR_SH_TYPE

sys.path.extend(['.', '..'])

from elftools.common.py3compat import bytes2str
from elftools.elf.elffile import ELFFile

def process_file(hash_key, filename):
    with open(filename, 'rb') as f:
        elffile = ELFFile(f)

        # Process sections
        sec_hdr_names_list = []
        for section in elffile.iter_sections():
            # Discard the following sections that dont add valuable
            # information to the hash
            # Below is a description of each one of the removed sections:
            #  .symtab - main symbol table used in compile-time linking or
            #            runtime debugging.
            #  .strtab - NULL-terminated strings of names of symbols in
            #            .symtab section.
            #  Both .symtab and strtab can be stripped from the ELF file
            #  without causing any issues in code execution. Please keep
            #  in mind that elftools doesn't consider .symtab and .strtab 
            #  as SHF_ALLOC sections.
            #
            #  .modinfo - module info section. It contains the kernel release
            #             number for which the module was built and it
            #             describes the form of the module's parameters.
            #             Mainly used by insmod. Since it varies depending
            #             on the kernel release number, we remove it from
            #             the valid sections group.
            #
            #  .gnu.linkonce.this_module - stores the struct module. This is
            #             used by the sys_init_module() during module
            #             initialization. Since this is used just for
            #             initialization purposes, we remove it from the
            #             valid sections group.
            #
            #  .note.gnu.build-id - section used to store a unique build id
            #             for the kernel and its modules. Since this changes
            #             for every different kernel built, we remove it from
            #             the valid sections group.
            if not (section.name.startswith(b'.gnu.linkonce.this_module') or
                    section.name.startswith(b'.modinfo') or
                    section.name.startswith(b'__versions') or
                    section.name.startswith(b'.strtab') or
                    section.name.startswith(b'.symtab') or
                    section.name.startswith(b'.note.gnu.build-id')):
                sec_hdr_names_list.append(section.name)

        # sort section names
        sec_hdr_names_list.sort()
        
        # Get sections in the sorted order and create a single canonicalized 
        # byte buffer to be hashed
        canonicalized_data = bytearray()
        for section_name in sec_hdr_names_list:
            section = elffile.get_section_by_name(section_name)
            if ((section['sh_flags'] & SH_FLAGS.SHF_ALLOC) and 
                (str(section['sh_type']) != 'SHT_NOBITS') and 
                (len(section.data()) > 0)):
                # For debugging purposes, use print stated below:
                #print('  ' + bytes2str(section_name) +
                #      '  ' + str(section['sh_type']) +
                #      '  ' + str(len(section.data())))
                canonicalized_data.extend(section.data())

        # Create the right content "hmac_sha256=yyy", yyy is the hmac sha256 value
        # worked out by python hashlib from the file argv[1]
        params = "hmac_sha256=" + hmac.new(binascii.unhexlify(hash_key), bytes(canonicalized_data), hashlib.sha256).hexdigest();
        sys.stdout.write(params)

# Process the given arguments
process_file(sys.argv[1], sys.argv[2])
