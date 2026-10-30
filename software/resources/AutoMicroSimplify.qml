import QtQuick
import QtQuick3D

Node {
    id: node

    // Resources
    property url textureData: "maps/textureData.png"
    property url textureData62: "maps/textureData62.png"
    property url textureData89: "maps/textureData89.png"
    Texture {
        id: _2_texture
        pivotV: 1
        rotationUV: 90
        scaleU: 0.007874015718698502
        scaleV: 0.001312335953116417
        generateMipmaps: true
        mipFilter: Texture.Linear
        source: node.textureData
    }
    Texture {
        id: _0_texture
        pivotV: 1
        rotationUV: 90
        scaleU: 0.007874015718698502
        scaleV: 0.001312335953116417
        generateMipmaps: true
        mipFilter: Texture.Linear
        source: node.textureData62
    }
    Texture {
        id: _1_texture
        pivotV: 1
        rotationUV: 90
        scaleU: 0.007874015718698502
        scaleV: 0.001312335953116417
        generateMipmaps: true
        mipFilter: Texture.Linear
        source: node.textureData89
    }
    PrincipledMaterial {
        id: m245245246_material
        objectName: "M245245246"
        baseColor: "#fff5f5f6"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: coolWhite_material
        objectName: "CoolWhite"
        baseColor: "#fff2f2ff"
        roughness: 0.42500001192092896
        normalMap: _1_texture
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: violet_material
        objectName: "Violet"
        baseColor: "#ff8359c5"
        roughness: 0.699999988079071
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m153153153_material
        objectName: "M153153153"
        baseColor: "#ff999999"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: red_material
        objectName: "Red"
        baseColor: "#ffb20000"
        roughness: 0.42500001192092896
        normalMap: _2_texture
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m253254255_material
        objectName: "M253254255"
        baseColor: "#fffdfeff"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m209204192_material
        objectName: "M209204192"
        baseColor: "#ffd1ccc0"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m01280_material
        objectName: "M01280"
        baseColor: "#ff008000"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m4859150_material
        objectName: "M4859150"
        baseColor: "#ff303b96"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m204204204_material
        objectName: "M204204204"
        baseColor: "#ffcccccc"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: gray_material
        objectName: "Gray"
        baseColor: "#ff696969"
        roughness: 0.699999988079071
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m190188186_material
        objectName: "M190188186"
        baseColor: "#ffbebcba"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m129129129_material
        objectName: "M129129129"
        baseColor: "#ff818181"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: white_material
        objectName: "White"
        baseColor: "#fff5f5f5"
        roughness: 0.10000000149011612
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m383838_material
        objectName: "M383838"
        baseColor: "#ff262626"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m228227207_material
        objectName: "M228227207"
        baseColor: "#ffe4e3cf"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m212224_material
        objectName: "M212224"
        baseColor: "#ff151618"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m212209201_material
        objectName: "M212209201"
        baseColor: "#ffd4d1c9"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m202209238_material
        objectName: "M202209238"
        baseColor: "#ffcad1ee"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: smoothYellow_material
        objectName: "SmoothYellow"
        baseColor: "#ffffff00"
        roughness: 0.800000011920929
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: darkGreen_material
        objectName: "DarkGreen"
        baseColor: "#ff0c9d0c"
        roughness: 0.699999988079071
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m255255255_material
        objectName: "M255255255"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: glossyBlack_material
        objectName: "GlossyBlack"
        baseColor: "#ff000000"
        metalness: 0.75
        roughness: 0.6600000262260437
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
        clearcoatAmount: 1
        clearcoatRoughnessAmount: 0.10000000149011612
    }
    PrincipledMaterial {
        id: blueWallPaintGlossy_material
        objectName: "BlueWallPaintGlossy"
        baseColor: "#ff1825f8"
        roughness: 0.47999998927116394
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m15810451_material
        objectName: "M15810451"
        baseColor: "#ff9e6833"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m47110151_material
        objectName: "M47110151"
        baseColor: "#ff2f6e97"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m35103185_material
        objectName: "M35103185"
        baseColor: "#ff2367b9"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m2437248_material
        objectName: "M2437248"
        baseColor: "#ff1825f8"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m224223219_material
        objectName: "M224223219"
        baseColor: "#ffe0dfdb"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m505050_material
        objectName: "M505050"
        baseColor: "#ff323232"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m047255_material
        objectName: "M047255"
        baseColor: "#ff002fff"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m7316984_material
        objectName: "M7316984"
        baseColor: "#ff49a954"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m163170173_material
        objectName: "M163170173"
        baseColor: "#ffa3aaad"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m262626_material
        objectName: "M262626"
        baseColor: "#ff1a1a1a"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m25500_material
        objectName: "M25500"
        baseColor: "#ffff0000"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m1651320_material
        objectName: "M1651320"
        baseColor: "#ffa58400"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m16012489_material
        objectName: "M16012489"
        baseColor: "#ffa07c59"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m767676_material
        objectName: "M767676"
        baseColor: "#ff4c4c4c"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m085121_material
        objectName: "M085121"
        baseColor: "#ff005579"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m111111_material
        objectName: "M111111"
        baseColor: "#ff0b0b0b"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m232222_material
        objectName: "M232222"
        baseColor: "#ff171616"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m18013989_material
        objectName: "M18013989"
        baseColor: "#ffb48b59"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m234234234_material
        objectName: "M234234234"
        baseColor: "#ffeaeaea"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m192192192_material
        objectName: "M192192192"
        baseColor: "#ffc0c0c0"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m373636_material
        objectName: "M373636"
        baseColor: "#ff252424"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m1158645_material
        objectName: "M1158645"
        baseColor: "#ff73562d"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m237237237_material
        objectName: "M237237237"
        baseColor: "#ffededed"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: default_material
        objectName: "Default"
        baseColor: "#ffbfbfbf"
        roughness: 0.699999988079071
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m210209199_material
        objectName: "M210209199"
        baseColor: "#ffd2d1c7"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m23217335_material
        objectName: "M23217335"
        baseColor: "#ffe8ad23"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m19914529_material
        objectName: "M19914529"
        baseColor: "#ffc7911d"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m7798129_material
        objectName: "M7798129"
        baseColor: "#ff4d6281"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m976955_material
        objectName: "M976955"
        baseColor: "#ff614537"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m87173113_material
        objectName: "M87173113"
        baseColor: "#ff57ad71"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m646464_material
        objectName: "M646464"
        baseColor: "#ff404040"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m229234237_material
        objectName: "M229234237"
        baseColor: "#ffe5eaed"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m80124105_material
        objectName: "M80124105"
        baseColor: "#ff507c69"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m20610938_material
        objectName: "M20610938"
        baseColor: "#ffce6d26"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m363634_material
        objectName: "M363634"
        baseColor: "#ff242422"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m969696_material
        objectName: "M969696"
        baseColor: "#ff606060"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: darkGray_material
        objectName: "DarkGray"
        baseColor: "#ff454545"
        roughness: 0.42500001192092896
        normalMap: _0_texture
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m219188126_material
        objectName: "M219188126"
        baseColor: "#ffdbbc7e"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m062186_material
        objectName: "M062186"
        baseColor: "#ff003eba"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m2182050_material
        objectName: "M2182050"
        baseColor: "#ffdacd00"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m2552040_material
        objectName: "M2552040"
        baseColor: "#ffffcc00"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m383737_material
        objectName: "M383737"
        baseColor: "#ff262525"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m251250245_material
        objectName: "M251250245"
        baseColor: "#fffbfaf5"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m176169152_material
        objectName: "M176169152"
        baseColor: "#ffb0a998"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m15613582_material
        objectName: "M15613582"
        baseColor: "#ff9c8752"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m200150100_material
        objectName: "M200150100"
        baseColor: "#ffc89664"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m247224153_material
        objectName: "M247224153"
        baseColor: "#fff7e099"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m229229229_material
        objectName: "M229229229"
        baseColor: "#ffe5e5e5"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m444_material
        objectName: "M444"
        baseColor: "#ff020202"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m246246243_material
        objectName: "M246246243"
        baseColor: "#fff6f6f3"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m215208192_material
        objectName: "M215208192"
        baseColor: "#ffd7d0c0"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m188188188_material
        objectName: "M188188188"
        baseColor: "#ffbcbcbc"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m000_material
        objectName: "M000"
        baseColor: "#ff000000"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m555_material
        objectName: "M555"
        baseColor: "#ff040404"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: anodizedBlack_material
        objectName: "AnodizedBlack"
        baseColor: "#ff0a0a0a"
        metalness: 1
        roughness: 0.699999988079071
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m252525_material
        objectName: "M252525"
        baseColor: "#ff191919"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m02550_material
        objectName: "M02550"
        baseColor: "#ff00ff00"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m15613581_material
        objectName: "M15613581"
        baseColor: "#ff9c8751"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m25523935_material
        objectName: "M25523935"
        baseColor: "#ffffef23"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m1888047_material
        objectName: "M1888047"
        baseColor: "#ffbc502f"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m18513872_material
        objectName: "M18513872"
        baseColor: "#ffb98a48"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: m178178178_material
        objectName: "M178178178"
        baseColor: "#ffb2b2b2"
        roughness: 0.8999999761581421
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }

    // Nodes:
    Node {
        id: root
        objectName: "ROOT"
        Model {
            id: solid1
            objectName: "Solid1"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/solid1_mesh.mesh"
            materials: [
                m80124105_material,
                m646464_material,
                m87173113_material,
                m976955_material,
                m7798129_material,
                m19914529_material,
                m23217335_material,
                m210209199_material,
                default_material,
                m237237237_material,
                m1158645_material,
                m373636_material,
                m192192192_material,
                m234234234_material,
                m18013989_material,
                m15810451_material,
                m111111_material,
                m085121_material,
                m767676_material,
                m16012489_material,
                m1651320_material,
                m25500_material,
                m262626_material,
                m163170173_material,
                m7316984_material,
                m047255_material,
                m505050_material,
                m224223219_material,
                m2437248_material,
                m35103185_material,
                m47110151_material,
                m232222_material,
                m178178178_material,
                m18513872_material,
                m1888047_material,
                m25523935_material,
                m15613581_material,
                m02550_material,
                m252525_material,
                anodizedBlack_material,
                m555_material,
                m000_material,
                m188188188_material,
                m215208192_material,
                m246246243_material,
                m444_material,
                m229234237_material,
                m247224153_material,
                m200150100_material,
                m15613582_material,
                m176169152_material,
                m251250245_material,
                m383737_material,
                m2552040_material,
                m2182050_material,
                m062186_material,
                m219188126_material,
                darkGray_material,
                m969696_material,
                m363634_material,
                m20610938_material,
                m255255255_material,
                m229229229_material,
                m202209238_material,
                m212209201_material,
                m212224_material,
                m228227207_material,
                m383838_material,
                white_material,
                m129129129_material,
                m190188186_material,
                gray_material,
                m204204204_material,
                m4859150_material,
                m01280_material,
                m209204192_material,
                m153153153_material,
                m253254255_material,
                m245245246_material
            ]
        }
        Model {
            id: srf1
            objectName: "Srf1"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf1_mesh.mesh"
            materials: [
                coolWhite_material
            ]
        }
        Model {
            id: srf2
            objectName: "Srf2"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf2_mesh.mesh"
            materials: [
                violet_material
            ]
        }
        Model {
            id: srf3
            objectName: "Srf3"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf3_mesh.mesh"
            materials: [
                red_material
            ]
        }
        Model {
            id: srf4
            objectName: "Srf4"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf4_mesh.mesh"
            materials: [
                coolWhite_material
            ]
        }
        Model {
            id: srf5
            objectName: "Srf5"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf5_mesh.mesh"
            materials: [
                coolWhite_material
            ]
        }
        Model {
            id: srf6
            objectName: "Srf6"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf6_mesh.mesh"
            materials: [
                coolWhite_material
            ]
        }
        Model {
            id: srf7
            objectName: "Srf7"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf7_mesh.mesh"
            materials: [
                coolWhite_material
            ]
        }
        Model {
            id: srf8
            objectName: "Srf8"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf8_mesh.mesh"
            materials: [
                violet_material
            ]
        }
        Model {
            id: srf9
            objectName: "Srf9"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf9_mesh.mesh"
            materials: [
                violet_material
            ]
        }
        Model {
            id: srf10
            objectName: "Srf10"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf10_mesh.mesh"
            materials: [
                violet_material
            ]
        }
        Model {
            id: srf11
            objectName: "Srf11"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf11_mesh.mesh"
            materials: [
                violet_material
            ]
        }
        Model {
            id: srf12
            objectName: "Srf12"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf12_mesh.mesh"
            materials: [
                red_material
            ]
        }
        Model {
            id: srf13
            objectName: "Srf13"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf13_mesh.mesh"
            materials: [
                red_material
            ]
        }
        Model {
            id: srf14
            objectName: "Srf14"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf14_mesh.mesh"
            materials: [
                red_material
            ]
        }
        Model {
            id: srf15
            objectName: "Srf15"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf15_mesh.mesh"
            materials: [
                red_material
            ]
        }
        Model {
            id: srf16
            objectName: "Srf16"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf16_mesh.mesh"
            materials: [
                blueWallPaintGlossy_material
            ]
        }
        Model {
            id: srf17
            objectName: "Srf17"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf17_mesh.mesh"
            materials: [
                glossyBlack_material
            ]
        }
        Model {
            id: srf18
            objectName: "Srf18"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf18_mesh.mesh"
            materials: [
                glossyBlack_material
            ]
        }
        Model {
            id: srf19
            objectName: "Srf19"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf19_mesh.mesh"
            materials: [
                red_material
            ]
        }
        Model {
            id: srf20
            objectName: "Srf20"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf20_mesh.mesh"
            materials: [
                red_material
            ]
        }
        Model {
            id: srf21
            objectName: "Srf21"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf21_mesh.mesh"
            materials: [
                blueWallPaintGlossy_material
            ]
        }
        Model {
            id: srf22
            objectName: "Srf22"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf22_mesh.mesh"
            materials: [
                red_material
            ]
        }
        Model {
            id: srf23
            objectName: "Srf23"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf23_mesh.mesh"
            materials: [
                red_material
            ]
        }
        Model {
            id: srf24
            objectName: "Srf24"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf24_mesh.mesh"
            materials: [
                glossyBlack_material
            ]
        }
        Model {
            id: srf25
            objectName: "Srf25"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf25_mesh.mesh"
            materials: [
                glossyBlack_material
            ]
        }
        Model {
            id: srf26
            objectName: "Srf26"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf26_mesh.mesh"
            materials: [
                blueWallPaintGlossy_material
            ]
        }
        Model {
            id: srf27
            objectName: "Srf27"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf27_mesh.mesh"
            materials: [
                glossyBlack_material
            ]
        }
        Model {
            id: srf28
            objectName: "Srf28"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf28_mesh.mesh"
            materials: [
                glossyBlack_material
            ]
        }
        Model {
            id: srf29
            objectName: "Srf29"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf29_mesh.mesh"
            materials: [
                red_material
            ]
        }
        Model {
            id: srf30
            objectName: "Srf30"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf30_mesh.mesh"
            materials: [
                red_material
            ]
        }
        Model {
            id: srf31
            objectName: "Srf31"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf31_mesh.mesh"
            materials: [
                darkGreen_material
            ]
        }
        Model {
            id: srf32
            objectName: "Srf32"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf32_mesh.mesh"
            materials: [
                darkGreen_material
            ]
        }
        Model {
            id: srf33
            objectName: "Srf33"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf33_mesh.mesh"
            materials: [
                smoothYellow_material
            ]
        }
        Model {
            id: srf34
            objectName: "Srf34"
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            source: "meshes/srf34_mesh.mesh"
            materials: [
                smoothYellow_material
            ]
        }
    }

    // Animations:
}
