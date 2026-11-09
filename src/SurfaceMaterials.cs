using System.Collections.Generic;

namespace ARS
{
    // The game's authored per-material tyre grip, keyed by the material's name hash - the value a shape test
    // returns for the ground under a point. The rows are the 213 materials in materials.dat order, taken from
    // the ars-fivem table that carries their names; the live table and its layout are proven in SP Physics
    // Tweaks (surface_speed_cap.h): 213 entries, stride 112 bytes, grip the first field of an entry.
    // A hash that is not here reads 1.0, so an unknown or renamed material never counts as loose ground.
    static class SurfaceMaterials
    {
        static readonly Dictionary<int, float> Grips = new Dictionary<int, float>
        {
            { unchecked((int)0xE4DF46D5), 1.0f }, { unchecked((int)0x9C3F221D), 1.0f },
            { unchecked((int)0xFF527A7F), 0.9f }, { unchecked((int)0x53E39AF5), 1.0f },
            { unchecked((int)0x0DD8089A), 1.0f }, { unchecked((int)0xCF4D943D), 0.9f },
            { unchecked((int)0x935CCFD7), 0.9f }, { unchecked((int)0x013F980D), 0.92f },
            { unchecked((int)0xDBC89F1B), 1.0f }, { unchecked((int)0x9DCB55CC), 1.0f },
            { unchecked((int)0x0BA85489), 0.8f }, { unchecked((int)0x0096E105), 1.0f },
            { unchecked((int)0x043A6103), 1.0f }, { unchecked((int)0xE9A64503), 1.0f },
            { unchecked((int)0x21D8740E), 0.9f }, { unchecked((int)0x2328A8CB), 1.0f },
            { unchecked((int)0x0B87C21F), 1.0f }, { unchecked((int)0x815FD901), 1.0f },
            { unchecked((int)0x97FC89BA), 0.75f }, { unchecked((int)0x8EEA76DD), 0.8f },
            { unchecked((int)0x3F791122), 0.75f }, { unchecked((int)0xB6B0644E), 0.9f },
            { unchecked((int)0xDAD5AB00), 0.9f }, { unchecked((int)0x151AF636), 0.75f },
            { unchecked((int)0xE56F97A1), 0.75f }, { unchecked((int)0xB20B0F5C), 0.45f },
            { unchecked((int)0xF6677BE5), 0.65f }, { unchecked((int)0xDFECD7B1), 0.4f },
            { unchecked((int)0xA14291E6), 0.65f }, { unchecked((int)0xF9786976), 0.4f },
            { unchecked((int)0xD43E04A8), 1.0f }, { unchecked((int)0x79C6B603), 0.75f },
            { unchecked((int)0x7346F38D), 0.85f }, { unchecked((int)0x74917A6D), 0.75f },
            { unchecked((int)0x8A011727), 0.75f }, { unchecked((int)0x09424A9F), 0.9f },
            { unchecked((int)0xD5FA0B8B), 0.9f }, { unchecked((int)0x90FB56A3), 0.9f },
            { unchecked((int)0x0C91EA71), 0.7f }, { unchecked((int)0xD42B8B0E), 0.7f },
            { unchecked((int)0x4CCDB75B), 0.7f }, { unchecked((int)0xDC02EABD), 0.7f },
            { unchecked((int)0xF100F0BE), 0.7f }, { unchecked((int)0xBE4A3483), 0.7f },
            { unchecked((int)0x24E77B36), 0.7f }, { unchecked((int)0xD461F6E5), 0.7f },
            { unchecked((int)0xF8005CB7), 0.7f }, { unchecked((int)0x72623394), 0.9f },
            { unchecked((int)0x4790F50D), 0.85f }, { unchecked((int)0xD07FDA8C), 0.7f },
            { unchecked((int)0x4DBCEFB5), 0.75f }, { unchecked((int)0x8707FB07), 0.8f },
            { unchecked((int)0xD1BC1641), 0.75f }, { unchecked((int)0x3038814F), 0.85f },
            { unchecked((int)0xB4021215), 0.85f }, { unchecked((int)0xE616BF5B), 1.0f },
            { unchecked((int)0x4ADF62DD), 1.0f }, { unchecked((int)0xAC067674), 1.0f },
            { unchecked((int)0x2ACF1553), 1.0f }, { unchecked((int)0xE4003252), 1.0f },
            { unchecked((int)0x6E310991), 1.0f }, { unchecked((int)0x52F91365), 1.0f },
            { unchecked((int)0x97D485F8), 1.0f }, { unchecked((int)0x0ADD9344), 1.0f },
            { unchecked((int)0x1F2B8444), 1.0f }, { unchecked((int)0x32AC1732), 1.0f },
            { unchecked((int)0xC45C8F33), 1.0f }, { unchecked((int)0xAB7B54F8), 1.0f },
            { unchecked((int)0x203E8CA7), 1.0f }, { unchecked((int)0x3FE87BC8), 0.8f },
            { unchecked((int)0xBFABBB92), 0.8f }, { unchecked((int)0xF9540006), 0.8f },
            { unchecked((int)0x2CB7A224), 0.8f }, { unchecked((int)0x2ED6D318), 0.8f },
            { unchecked((int)0xD07084D5), 0.8f }, { unchecked((int)0xE8F6F419), 0.8f },
            { unchecked((int)0xAACFA560), 0.8f }, { unchecked((int)0xD4703043), 0.8f },
            { unchecked((int)0xCFE7CAC9), 0.8f }, { unchecked((int)0x71DBCB62), 0.8f },
            { unchecked((int)0x0FBC9F9C), 0.8f }, { unchecked((int)0x0DDCF165), 0.9f },
            { unchecked((int)0x557D8CEE), 0.9f }, { unchecked((int)0xB739B2E0), 0.8f },
            { unchecked((int)0xB847E8F3), 0.9f }, { unchecked((int)0xDFD38795), 0.8f },
            { unchecked((int)0x11EBE399), 0.8f }, { unchecked((int)0x6D3AFF29), 0.85f },
            { unchecked((int)0x6E742E8D), 0.8f }, { unchecked((int)0x63E4678E), 0.8f },
            { unchecked((int)0x5908BC09), 0.85f }, { unchecked((int)0x3E9E8413), 0.8f },
            { unchecked((int)0xE32C66CC), 0.9f }, { unchecked((int)0x4298D405), 1.0f },
            { unchecked((int)0xDBDF966D), 1.0f }, { unchecked((int)0xD501EA57), 0.9f },
            { unchecked((int)0x90543A4D), 0.8f }, { unchecked((int)0x25921A02), 0.8f },
            { unchecked((int)0xF614FA4D), 0.8f }, { unchecked((int)0x16045441), 0.8f },
            { unchecked((int)0x1951F94C), 0.8f }, { unchecked((int)0x7196A863), 0.8f },
            { unchecked((int)0x2C44BFFB), 0.8f }, { unchecked((int)0x162A5452), 0.8f },
            { unchecked((int)0x1355B189), 0.85f }, { unchecked((int)0xB066A92E), 0.8f },
            { unchecked((int)0xDD89CA9B), 0.8f }, { unchecked((int)0xAC7F9D74), 0.8f },
            { unchecked((int)0x743461D1), 1.0f }, { unchecked((int)0xC73DC975), 1.0f },
            { unchecked((int)0x37A9581E), 0.9f }, { unchecked((int)0x0F6323AA), 0.9f },
            { unchecked((int)0x65371742), 0.9f }, { unchecked((int)0x7E3147D0), 0.9f },
            { unchecked((int)0xDCDBF722), 0.9f }, { unchecked((int)0x8725491A), 0.9f },
            { unchecked((int)0x083F9467), 0.9f }, { unchecked((int)0x846F5EBB), 0.9f },
            { unchecked((int)0x6A87DF76), 0.9f }, { unchecked((int)0x9152D650), 0.9f },
            { unchecked((int)0x642AFDE0), 0.9f }, { unchecked((int)0xF3BA433B), 0.9f },
            { unchecked((int)0x92A90A21), 0.9f }, { unchecked((int)0xAA6387F8), 0.9f },
            { unchecked((int)0xC8B9B6F1), 0.9f }, { unchecked((int)0xCADD9676), 0.9f },
            { unchecked((int)0xCA1D58F8), 0.9f }, { unchecked((int)0x8E791578), 0.9f },
            { unchecked((int)0xD8386279), 0.9f }, { unchecked((int)0xC514BC40), 1.0f },
            { unchecked((int)0xE4A161A0), 1.0f }, { unchecked((int)0x99C941FC), 0.9f },
            { unchecked((int)0xC8E99AC8), 0.8f }, { unchecked((int)0x63E0CEAB), 1.0f },
            { unchecked((int)0x1C83D9D0), 1.0f }, { unchecked((int)0x6A3C1CD6), 1.0f },
            { unchecked((int)0xCB4DAA4F), 1.0f }, { unchecked((int)0x448C6991), 0.8f },
            { unchecked((int)0x8D191712), 0.8f }, { unchecked((int)0x20D45096), 1.0f },
            { unchecked((int)0xA5AB2856), 0.5f }, { unchecked((int)0x5D5CFB0A), 0.5f },
            { unchecked((int)0xE9DB60EB), 1.0f }, { unchecked((int)0x59C981F2), 0.7f },
            { unchecked((int)0x66D49191), 0.7f }, { unchecked((int)0x44E56D52), 0.7f },
            { unchecked((int)0xCD14181F), 0.7f }, { unchecked((int)0xA970B0B6), 0.9f },
            { unchecked((int)0x0FD4D2CE), 0.9f }, { unchecked((int)0x183BD8A4), 0.9f },
            { unchecked((int)0x38D72531), 0.7f }, { unchecked((int)0x1F20EC2B), 0.7f },
            { unchecked((int)0x17C1508E), 0.7f }, { unchecked((int)0xDA325F68), 0.7f },
            { unchecked((int)0xB82041E7), 0.7f }, { unchecked((int)0x1A8C87DC), 0.7f },
            { unchecked((int)0xB4E9AB9A), 0.7f }, { unchecked((int)0x3825C8C1), 0.7f },
            { unchecked((int)0x4E03F47D), 0.7f }, { unchecked((int)0x14B201DA), 0.7f },
            { unchecked((int)0x29E12C38), 0.7f }, { unchecked((int)0x6028BB31), 0.7f },
            { unchecked((int)0xB7068898), 0.7f }, { unchecked((int)0xC90EE99D), 0.7f },
            { unchecked((int)0x3FD836C7), 0.7f }, { unchecked((int)0x9BE4E62A), 0.7f },
            { unchecked((int)0x6B720544), 0.7f }, { unchecked((int)0x72AA0A99), 0.7f },
            { unchecked((int)0x56C231E2), 0.7f }, { unchecked((int)0x3B6765C5), 0.7f },
            { unchecked((int)0x84533F51), 0.7f }, { unchecked((int)0xD11E6E71), 0.7f },
            { unchecked((int)0x586FED0A), 0.9f }, { unchecked((int)0x600D74FE), 0.9f },
            { unchecked((int)0x7E35E1D8), 1.0f }, { unchecked((int)0xA6292B20), 1.0f },
            { unchecked((int)0x36C3763D), 0.8f }, { unchecked((int)0xAA43D6BA), 0.8f },
            { unchecked((int)0xD6D5DD12), 0.8f }, { unchecked((int)0x9FBEC8B7), 1.0f },
            { unchecked((int)0xC831DD47), 0.75f }, { unchecked((int)0xB9A9AD0A), 1.0f },
            { unchecked((int)0xE85DEDF4), 1.0f }, { unchecked((int)0x34DA9613), 0.75f },
            { unchecked((int)0x5FA3EBA5), 0.75f }, { unchecked((int)0x745014F1), 0.75f },
            { unchecked((int)0x9E8B6967), 0.75f }, { unchecked((int)0x593A5EC6), 0.75f },
            { unchecked((int)0x8300B252), 0.75f }, { unchecked((int)0xADD587FF), 0.75f },
            { unchecked((int)0xD794DB7D), 0.75f }, { unchecked((int)0x8A3040B1), 0.75f },
            { unchecked((int)0x38281E4E), 0.75f }, { unchecked((int)0x9258D2AE), 0.75f },
            { unchecked((int)0xC18CB119), 0.75f }, { unchecked((int)0x6BDC85B6), 0.75f },
            { unchecked((int)0x9D1BE834), 0.75f }, { unchecked((int)0x1BF8E5EC), 0.75f },
            { unchecked((int)0x0D33C862), 0.75f }, { unchecked((int)0xD776DCE9), 0.75f },
            { unchecked((int)0xECC1877E), 0.75f }, { unchecked((int)0x65A17928), 0.75f },
            { unchecked((int)0x00512AB5), 0.75f }, { unchecked((int)0x15AF5571), 0.75f },
            { unchecked((int)0x23ECF1EC), 0.75f }, { unchecked((int)0x46FE381A), 0.75f },
            { unchecked((int)0x95D0D5BE), 0.75f }, { unchecked((int)0x4EE4C7E7), 0.75f },
            { unchecked((int)0x5CAE637A), 0.75f }, { unchecked((int)0x0AA7BFB6), 0.75f },
            { unchecked((int)0xF86A1B3B), 0.75f }, { unchecked((int)0x0B02406B), 0.75f },
            { unchecked((int)0x28D7FD2A), 0.75f },
        };

        public static bool TryGrip(int materialHash, out float grip)
        {
            return Grips.TryGetValue(materialHash, out grip);
        }

        public static float Grip(int materialHash)
        {
            float grip;
            return Grips.TryGetValue(materialHash, out grip) ? grip : 1f;
        }
    }
}
