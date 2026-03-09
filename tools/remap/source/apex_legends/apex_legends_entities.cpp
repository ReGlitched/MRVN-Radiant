/* -------------------------------------------------------------------------------

   Copyright (C) 2022-2025 MRVN-Radiant and contributors.
   For a list of contributors, see the accompanying CONTRIBUTORS file.

   This file is part of MRVN-Radiant.

   MRVN-Radiant is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   MRVN-Radiant is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with GtkRadiant; if not, write to the Free Software
   Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

   ------------------------------------------------------------------------------- */

/*
    Apex Legends Entity Lumps

    This file handles entity-related BSP lumps:
    - Entities lump (0x00): Entity definitions
    - GameLump: Static props and paths
*/

#include "../remap.h"
#include "../bspfile_abstract.h"
#include "../model.h"
#include <cstring>

/*
    EmitEntity()
    Saves an entity into its corresponding .ent file or the lump in the .bsp
    
    Entities are categorized into different files based on their classname:
    - env: lights, fog, sky, color entities
    - fx: particle effects
    - script: info_target, prop_dynamic, trigger_hurt
    - spawn: info_* entities
    - BSP entity lump: all other entities
*/
void ApexLegends::EmitEntity(const entity_t &e) {
    StringOutputStream data;
    data << "{\n";
    for (const epair_t& pair : e.epairs) {
        data << "\"" << pair.key.c_str() << "\" \"" << pair.value.c_str() << "\"\n";
    }
    data << "}\n";

    std::vector<char> str = { data.begin(), data.end() };

    // env
    if (striEqualPrefix(e.valueForKey("classname"), "light")
     || striEqualPrefix(e.valueForKey("classname"), "color")
     || striEqualPrefix(e.valueForKey("classname"), "fog")
     || striEqualPrefix(e.valueForKey("classname"), "env")
     || striEqualPrefix(e.valueForKey("classname"), "sky")) {
        Titanfall::Ent::env.insert(Titanfall::Ent::env.end(), str.begin(), str.end());
    // fx
    } else if(striEqualPrefix(e.valueForKey("classname"), "info_particle")) {
        Titanfall::Ent::fx.insert(Titanfall::Ent::fx.end(), str.begin(), str.end());
    // script
    } else if (striEqualPrefix(e.valueForKey("classname"), "info_target")
            || striEqualPrefix(e.valueForKey("classname"), "prop_dynamic")
            || striEqualPrefix(e.valueForKey("classname"), "trigger_hurt")
            || striEqualPrefix(e.valueForKey("classname"), "zipline")) {
        Titanfall::Ent::script.insert(Titanfall::Ent::script.end(), str.begin(), str.end());
    // snd
    // spawn
    } else if (striEqualPrefix(e.valueForKey("classname"), "info_")) {
        Titanfall::Ent::spawn.insert(Titanfall::Ent::spawn.end(), str.begin(), str.end());
    // bsp entity lump
    } else {
        Titanfall::Bsp::entities.insert(Titanfall::Bsp::entities.end(), str.begin(), str.end());
    }
}

/*
    EmitStaticProp()
    Emits a static prop into the GameLump

    Static props are compiled into the GameLump section of the BSP.
    Each prop references a model path and has position/rotation data.
*/
void ApexLegends::EmitStaticProp(entity_t &e) {
    const char *model = e.valueForKey("model");
    int16_t pathIdx = -1;
    MinMax minmax;

    std::vector<const AssMeshWalker*> meshes = LoadModelWalker( model, 0 );
    if(meshes.empty()) {
        Sys_Warning("Failed to load model: %s\n", model);
        return;
    }

    // Strip "models/" prefix — Apex game lump paths start at "mdl/"
    const char *lumpPath = model;
    if (striEqualPrefix(lumpPath, "models/"))
        lumpPath += 7;

    Sys_FPrintf(SYS_VRB, "  Emitting static prop: %s\n", lumpPath);

    // Find existing path or add new one
    for(std::size_t i = 0; i < ApexLegends::Bsp::gameLumpPaths.size(); i++) {
        Titanfall::GameLumpPath_t &path = ApexLegends::Bsp::gameLumpPaths.at(i);

        if(!string_compare_nocase(path.path, lumpPath)) {
            pathIdx = i;
            break;
        }
    }

    if(pathIdx == -1) {
        ApexLegends::Bsp::gameLumpPathHeader.numPaths++;
        Titanfall::GameLumpPath_t &path = ApexLegends::Bsp::gameLumpPaths.emplace_back();
        strncpy(path.path, lumpPath, 127);
        path.path[127] = '\0';
        pathIdx = ApexLegends::Bsp::gameLumpPaths.size() - 1;
    }

    // Increment prop counts
    // All remap-compiled props are treated as opaque for now
    ApexLegends::Bsp::gameLumpPropHeader.numStaticProps++;
    ApexLegends::Bsp::gameLumpPropHeader.numOpaqueProps++;
    ApexLegends::Bsp::gameLumpPropHeader.firstTransProp++;

    ApexLegends::GameLumpProp_t &prop = ApexLegends::Bsp::gameLumpProps.emplace_back();
    memset(&prop, 0, sizeof(prop));

    Vector3 origin;
    Vector3 angles;
    if(!e.read_keyvalue(origin, "origin")) {
        origin.x() = 0.0f;
        origin.y() = 0.0f;
        origin.z() = 0.0f;
    }
    if(!e.read_keyvalue(angles, "angles")) {
        angles.x() = 0.0f;
        angles.y() = 0.0f;
        angles.z() = 0.0f;
    }

    prop.origin = origin;
    prop.angles = angles;

    // floatForKey/intForKey treat all arguments as key names, not defaults
    float scale = e.floatForKey("modelscale");
    if (scale == 0.0f) scale = 1.0f;
    prop.scale = scale;

    prop.modelName = pathIdx;

    int solid = e.intForKey("solid");
    if (solid == 0) solid = 6;  // Default: SOLID_VPHYSICS
    prop.solid = (uint8_t)solid;

    prop.flags = (uint8_t)e.intForKey("staticPropFlags") | 0x04;  // Bit 2 always set (engine requires for staticPropFlags 0x40)
    prop.skin = (uint16_t)e.intForKey("skin");
    prop.envCubemap = 0xFFFF;       // Use default cubemap

    float fadeDist = e.floatForKey("fademindist");
    if (fadeDist == 0.0f) fadeDist = -1.0f;  // -1 = engine calculates default
    prop.fadeDist = fadeDist;
    // Use prop origin as lighting sample point; set flags bit 1 for custom origin
    prop.flags |= 0x02;
    prop.lightingOrigin = origin;
    prop.diffuseModulation[0] = 255;  // R
    prop.diffuseModulation[1] = 255;  // G
    prop.diffuseModulation[2] = 255;  // B
    prop.diffuseModulation[3] = 255;  // A
    prop.precompiledWind = 0;         // No wind
    prop.setDressLevel = 0;           // Default LOD level
}

/*
    SetupGameLump()
    Initializes the GameLump header data
*/
void ApexLegends::SetupGameLump() {
    ApexLegends::Bsp::gameLumpHeader.version = 1;
    memcpy(ApexLegends::Bsp::gameLumpHeader.ident, "prps", 4);
    ApexLegends::Bsp::gameLumpHeader.gameConst = 3080192;  // 0x002F0000 = bspVersion(47) << 16

    ApexLegends::Bsp::gameLumpPathHeader.numPaths = 0;

    ApexLegends::Bsp::gameLumpPropHeader.numStaticProps = 0;
    ApexLegends::Bsp::gameLumpPropHeader.numOpaqueProps = 0;
    ApexLegends::Bsp::gameLumpPropHeader.firstTransProp = 0;
}
