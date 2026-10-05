using System.Collections;
using System.Numerics;
using HkxSharp;
using HkxSimSharp.Cloth.Data;
using HkxSimSharp.Cloth.Data.Animation;
using HkxSimSharp.Cloth.Data.Collidables;
using HkxSimSharp.Cloth.Data.Constraints;
using HkxSimSharp.Cloth.Data.Operators;

namespace HkxSimSharp.Cloth;

/// <summary>Converts the generic object graph decoded by HkxSharp into simulation-ready cloth data.</summary>
public static class HkxClothAdapter
{
    public static HkxClothDocument Decode(HkxFile file)
    {
        ArgumentNullException.ThrowIfNull(file);
        HkObject cloth = file.OfClass("hclClothContainer").FirstOrDefault()
            ?? throw new InvalidDataException("The HKX file does not contain an hclClothContainer.");
        HkObject? animation = file.OfClass("hkaAnimationContainer").FirstOrDefault();
        return Decode(cloth, animation);
    }

    public static HkxClothDocument Decode(HkObject clothContainer, HkObject? animationContainer = null)
    {
        ArgumentNullException.ThrowIfNull(clothContainer);
        return new HkxClothDocument(ReadContainer(clothContainer),
            animationContainer is null ? null : ReadAnimations(animationContainer));
    }

    static HclClothContainer ReadContainer(HkObject source)
    {
        var result = new HclClothContainer();
        result.Collidables.AddRange(Items(source, "collidables").OfType<HkObject>().Select(ReadCollidable));
        result.ClothDatas.AddRange(Items(source, "clothDatas").OfType<HkObject>().Select(ReadCloth));
        return result;
    }

    static HclClothData ReadCloth(HkObject source)
    {
        var result = new HclClothData
        {
            Name = String(source, "name"),
            TargetPlatform = UInt32(source, "targetPlatform")
        };
        result.SimClothDatas.AddRange(Items(source, "simClothDatas").OfType<HkObject>().Select(ReadSimCloth));
        result.BufferDefinitions.AddRange(Items(source, "bufferDefinitions").OfType<HkObject>().Select(ReadBuffer));
        result.TransformSetDefinitions.AddRange(Items(source, "transformSetDefinitions").OfType<HkObject>().Select(ReadTransformSet));
        foreach (HkObject op in Items(source, "operators").OfType<HkObject>())
            if (ReadOperator(op) is { } decoded) result.Operators.Add(decoded);
        return result;
    }

    static HclSimClothData ReadSimCloth(HkObject source)
    {
        object? simulationInfo = Field(source, "simulationInfo");
        var particles = Items(source, "particleDatas").Select(p => new HclParticleData(
            Float(p, "mass"), Float(p, "invMass"), Float(p, "radius"), Float(p, "friction"))).ToArray();
        var result = new HclSimClothData
        {
            Name = String(source, "name"),
            Gravity = Vector3(Field(simulationInfo, "gravity")),
            GlobalDamping = Float(simulationInfo, "globalDampingPerSecond", 0.1f),
            Particles = particles,
            FixedParticles = Array<ushort>(source, "fixedParticles"),
            Poses = Items(source, "simClothPoses").OfType<HkObject>().Select(p => new HclSimClothPose
            {
                Name = String(p, "name"),
                Positions = Vector3Array(Field(p, "positions"))
            }).ToArray(),
            ConstraintSets = Items(source, "staticConstraintSets").OfType<HkObject>().Select(ReadConstraint).ToArray(),
            Collidables = Items(source, "perInstanceCollidables").OfType<HkObject>().Select(ReadCollidable).ToArray(),
            StaticCollisionMasks = Array<uint>(source, "staticCollisionMasks"),
            TotalMass = Float(source, "totalMass"),
            TriangleIndices = Array<ushort>(source, "triangleIndices"),
            PinchDetectionEnabled = Bool(source, "pinchDetectionEnabled"),
            PerParticlePinchDetectionEnabled = Array<bool>(source, "perParticlePinchDetectionEnabledFlags"),
            CollidablePinchingData = Items(source, "collidablePinchingDatas").Select(p =>
                new HclCollidablePinchingData(Bool(p, "pinchDetectionEnabled"),
                    SByte(p, "pinchDetectionPriority"), Float(p, "pinchDetectionRadius"))).ToArray()
        };

        object? map = Field(source, "collidableTransformMap");
        result.CollidableTransformSetIndex = Int32(map, "transformSetIndex", -1);
        result.CollidableTransformIndices = Array<uint>(map, "transformIndices");
        result.CollidableTransformOffsets = MatrixArray(Field(map, "offsets"));

        object? virtualPoints = Field(source, "virtualCollisionPointsData");
        result.NumVirtualCollisionPoints = UInt16(virtualPoints, "numVCPoints");

        object? transfer = Field(source, "transferMotionData");
        result.TransferMotion = new HclTransferMotionData
        {
            TransformSetIndex = UInt32(transfer, "transformSetIndex"),
            TransformIndex = UInt32(transfer, "transformIndex"),
            TransferTranslationMotion = Bool(transfer, "transferTranslationMotion", true),
            MinTranslationSpeed = Float(transfer, "minTranslationSpeed", 0.05f),
            MaxTranslationSpeed = Float(transfer, "maxTranslationSpeed", 3f),
            MinTranslationBlend = Float(transfer, "minTranslationBlend", 0.85f),
            MaxTranslationBlend = Float(transfer, "maxTranslationBlend", 0.35f),
            TransferRotationMotion = Bool(transfer, "transferRotationMotion", true),
            MinRotationSpeed = Float(transfer, "minRotationSpeed", 0.2f),
            MaxRotationSpeed = Float(transfer, "maxRotationSpeed", 6f),
            MinRotationBlend = Float(transfer, "minRotationBlend", 0.85f),
            MaxRotationBlend = Float(transfer, "maxRotationBlend", 0.35f)
        };
        return result;
    }

    static HclConstraintSet ReadConstraint(HkObject source)
    {
        string name = String(source, "name");
        uint id = UInt32(Field(source, "constraintId"));
        uint type = UInt32(source, "type");
        HclConstraintSet result = source.ClassName switch
        {
            "hclStandardLinkConstraintSet" => new HclStandardLinkConstraintSet
            {
                Links = Items(source, "links").Select(ReadStandardLink).ToArray()
            },
            "hclStretchLinkConstraintSet" => new HclStretchLinkConstraintSet
            {
                Links = Items(source, "links").Select(ReadStandardLink).ToArray()
            },
            "hclCompressibleLinkConstraintSet" => new HclCompressibleLinkConstraintSet
            {
                Links = Items(source, "links").Select(x => new CompressibleLink(
                    UInt16(x, "particleA"), UInt16(x, "particleB"), Float(x, "restLength"),
                    Float(x, "compressionLength"), Float(x, "stiffness"))).ToArray()
            },
            "hclBendLinkConstraintSet" => new HclBendLinkConstraintSet
            {
                Links = Items(source, "links").Select(x => new BendLink(
                    UInt16(x, "particleA"), UInt16(x, "particleB"), Float(x, "bendMinLength"),
                    Float(x, "stretchMaxLength"), Float(x, "bendStiffness"), Float(x, "stretchStiffness"))).ToArray()
            },
            "hclBendStiffnessConstraintSet" => new HclBendStiffnessConstraintSet
            {
                Links = Items(source, "links").Select(x => new BendStiffnessLink(
                    Float(x, "weightA"), Float(x, "weightB"), Float(x, "weightC"), Float(x, "weightD"),
                    Float(x, "bendStiffness"), Float(x, "restCurvature"), UInt16(x, "particleA"),
                    UInt16(x, "particleB"), UInt16(x, "particleC"), UInt16(x, "particleD"))).ToArray(),
                MaxRestPoseHeightSq = Float(source, "maxRestPoseHeightSq"),
                ClampBendStiffness = Bool(source, "clampBendStiffness"),
                UseRestPoseConfig = Bool(source, "useRestPoseConfig")
            },
            "hclLocalRangeConstraintSet" => ReadLocalRange(source),
            _ => new HclUnsupportedConstraintSet()
        };
        result.Name = name;
        result.ConstraintId = id;
        result.Type = type;
        return result;
    }

    static HclLocalRangeConstraintSet ReadLocalRange(HkObject source)
    {
        var items = Items(source, "localConstraints").ToArray();
        bool hasPerConstraintStiffness = false;
        if (items.Length == 0)
        {
            items = Items(source, "localStiffnessConstraints").ToArray();
            hasPerConstraintStiffness = true;
        }
        return new HclLocalRangeConstraintSet
        {
            LocalConstraints = items.Select(x => new LocalRangeConstraint(
                UInt16(x, "particleIndex"), UInt16(x, "referenceVertex"), Float(x, "shapeRadius"),
                Float(x, "maxNormalDistance"), Float(x, "minNormalDistance"),
                hasPerConstraintStiffness ? Float(x, "stiffness", 1f) : 1f)).ToArray(),
            ReferenceMeshBufferIdx = UInt32(source, "referenceMeshBufferIdx"),
            Stiffness = Float(source, "stiffness", 1f),
            ShapeType = UInt32(source, "shapeType")
        };
    }

    static StandardLink ReadStandardLink(object? source) => new(
        UInt16(source, "particleA"), UInt16(source, "particleB"),
        Float(source, "restLength"), Float(source, "stiffness"));

    static HclOperator? ReadOperator(HkObject source)
    {
        HclOperator? result = source.ClassName switch
        {
            "hclSimulateOperator" => ReadSimulateOperator(source),
            "hclMoveParticlesOperator" => new HclMoveParticlesOperator
            {
                VertexParticlePairs = Items(source, "vertexParticlePairs").Select(x =>
                    new VertexParticlePair(UInt16(x, "vertexIndex"), UInt16(x, "particleIndex"))).ToArray(),
                SimClothIndex = UInt32(source, "simClothIndex"),
                RefBufferIndex = UInt32(source, "refBufferIdx")
            },
            "hclGatherAllVerticesOperator" => new HclGatherAllVerticesOperator
            {
                VertexInputFromVertexOutput = Array<short>(source, "vertexInputFromVertexOutput"),
                InputBufferIndex = UInt32(source, "inputBufferIdx"),
                OutputBufferIndex = UInt32(source, "outputBufferIdx"),
                GatherNormals = Bool(source, "gatherNormals"),
                PartialGather = Bool(source, "partialGather")
            },
            "hclSimpleMeshBoneDeformOperator" => new HclSimpleMeshBoneDeformOperator
            {
                InputBufferIndex = UInt32(source, "inputBufferIdx"),
                OutputTransformSetIndex = UInt32(source, "outputTransformSetIdx"),
                TriangleBonePairs = Items(source, "triangleBonePairs").Select(x =>
                    new TriangleBonePair(UInt16(x, "boneOffset"), UInt16(x, "triangleOffset"))).ToArray(),
                LocalBoneTransforms = MatrixArray(Field(source, "localBoneTransforms")),
                BoneAxis = UInt32(source, "boneAxis")
            },
            _ when source.ClassName.StartsWith("hclObjectSpaceSkin", StringComparison.Ordinal) => ReadSkinOperator(source),
            _ when source.ClassName.StartsWith("hclBoneSpaceSkin", StringComparison.Ordinal) => ReadBoneSpaceSkinOperator(source),
            _ => null
        };
        if (result is not null)
        {
            result.Name = String(source, "name");
            result.OperatorId = UInt32(source, "operatorID");
            result.Type = UInt32(source, "type");
        }
        return result;
    }

    static HclSimulateOperator ReadSimulateOperator(HkObject source)
    {
        object? config = Items(source, "simulateOpConfigs").FirstOrDefault();
        return new HclSimulateOperator
        {
            SimClothIndex = UInt32(source, "simClothIndex"),
            ConstraintExecution = Array<int>(config, "constraintExecution"),
            InstanceCollidablesUsed = Array<bool>(config, "instanceCollidablesUsed"),
            SubSteps = Byte(config, "subSteps", 1),
            NumberOfSolveIterations = Byte(config, "numberOfSolveIterations", 1),
            UseAllInstanceCollidables = Bool(config, "useAllInstanceCollidables", true),
            AdaptConstraintStiffness = Bool(config, "adaptConstraintStiffness")
        };
    }

    static HclObjectSpaceSkinOperator ReadSkinOperator(HkObject source) => new()
    {
        BoneFromSkinMeshTransforms = MatrixArray(Field(source, "boneFromSkinMeshTransforms")),
        TransformSubset = Array<ushort>(source, "transformSubset"),
        OutputBufferIndex = UInt32(source, "outputBufferIndex"),
        TransformSetIndex = UInt32(source, "transformSetIndex"),
        Deformer = ReadDeformer(Field(source, "objectSpaceDeformer"), source)
    };

    /// <summary>
    /// <c>hclBoneSpaceSkin*Operator</c>: the skin Rope_1, Pod_C and Bag use in place of the object-space one. Each vertex
    /// holds a position in the space of the bone it follows, so the skinned position is that position carried by the
    /// bone's current transform - which is what the object-space path computes when its bone-from-skin-mesh
    /// matrices are identity, so the operator is read into the same type with those. Dropped, the piece's reference
    /// buffer is never written, and the anchors that read it sit at the origin.
    /// </summary>
    static HclObjectSpaceSkinOperator ReadBoneSpaceSkinOperator(HkObject source)
    {
        ushort[] subset = Array<ushort>(source, "transformSubset");
        return new HclObjectSpaceSkinOperator
        {
            BoneFromSkinMeshTransforms = Enumerable.Repeat(Matrix4x4.Identity, subset.Length).ToArray(),
            TransformSubset = subset,
            OutputBufferIndex = UInt32(source, "outputBufferIndex"),
            TransformSetIndex = UInt32(source, "transformSetIndex"),
            Deformer = ReadDeformer(Field(source, "boneSpaceDeformer"), source, weightsInLocalW: true)
        };
    }

    static HclObjectSpaceDeformer ReadDeformer(object? source, HkObject owner, bool weightsInLocalW = false)
    {
        string[] names = ["oneBlendEntries", "twoBlendEntries", "threeBlendEntries", "fourBlendEntries",
            "fiveBlendEntries", "sixBlendEntries", "sevenBlendEntries", "eightBlendEntries"];
        var blocks = new HclObjectSpaceBlendBlock[8][];
        for (int influence = 1; influence <= 8; influence++)
        {
            int n = influence;
            blocks[influence - 1] = Items(source, names[influence - 1]).Select(x => new HclObjectSpaceBlendBlock
            {
                InfluenceCount = n,
                VertexIndices = Array<ushort>(x, "vertexIndices"),
                BoneIndices = Array<ushort>(x, "boneIndices"),
                BoneWeights = DecodeWeights(Field(x, "boneWeights"), n)
            }).ToArray();
            if (influence == 1)
                foreach (var block in blocks[0]) block.BoneWeights = Enumerable.Repeat(1f, block.BoneIndices.Length).ToArray();
        }
        return new HclObjectSpaceDeformer
        {
            BlendBlocks = blocks,
            ControlBytes = Array<byte>(source, "controlBytes"),
            PackedLocalPositions = Items(owner, "localPs").Select(x => Vector3Array(Field(x, "localPosition"))).ToArray(),
            UnpackedLocalPositions = Items(owner, "localUnpackedPs").Select(x => Vector3Array(Field(x, "localPosition"))).ToArray(),
            WeightsInLocalW = weightsInLocalW,
            PackedLocalWeights = weightsInLocalW ? Items(owner, "localPs").Select(x => LocalW(Field(x, "localPosition"))).ToArray() : [],
            UnpackedLocalWeights = weightsInLocalW ? Items(owner, "localUnpackedPs").Select(x => LocalW(Field(x, "localPosition"))).ToArray() : [],
            StartVertexIndex = UInt16(source, "startVertexIndex"),
            EndVertexIndex = UInt16(source, "endVertexIndex"),
            PartialWrite = Bool(source, "partialWrite")
        };
    }

    static float[] DecodeWeights(object? value, int influences) => value switch
    {
        byte[] a => a.Select(x => x / 255f).ToArray(),
        ushort[] a => a.Select(x => x / 65535f).ToArray(),
        float[] a => a,
        _ => System.Array.Empty<float>()
    };

    static HclCollidable ReadCollidable(HkObject source) => new()
    {
        Name = String(source, "name"),
        Transform = Matrix(Field(source, "transform")),
        Shape = Field(source, "shape") is HkObject shape ? ReadShape(shape) : null,
        PinchDetectionRadius = Float(source, "pinchDetectionRadius"),
        PinchDetectionPriority = SByte(source, "pinchDetectionPriority"),
        PinchDetectionEnabled = Bool(source, "pinchDetectionEnabled"),
        VirtualCollisionPointCollisionEnabled = Bool(source, "virtualCollisionPointCollisionEnabled"),
        Enabled = Bool(source, "enabled", true)
    };

    static HclShape? ReadShape(HkObject source) => source.ClassName switch
    {
        "hclCapsuleShape" => new HclCapsuleShape
        {
            Start = Vector3(Field(source, "start")),
            End = Vector3(Field(source, "end")),
            Direction = Vector3(Field(source, "dir")),
            Radius = Float(source, "radius"),
            CapLenSqrdInv = Float(source, "capLenSqrdInv")
        },
        "hclSphereShape" => new HclSphereShape
        {
            Center = Vector3(Field(source, "center")),
            Radius = Float(source, "radius")
        },
        "hclTaperedCapsuleShape" => new HclTaperedCapsuleShape
        {
            Small = Vector3(Field(source, "small")),
            Big = Vector3(Field(source, "big")),
            SmallRadius = Float(source, "smallRadius"),
            BigRadius = Float(source, "bigRadius")
        },
        "hclPlaneShape" => new HclPlaneShape { PlaneEquation = Vector4(Field(source, "plane")) },
        _ => null
    };

    static HclBufferDefinition ReadBuffer(HkObject source) => new()
    {
        MeshName = String(source, "meshName"),
        BufferName = String(source, "bufferName"),
        Type = UInt32(source, "type"),
        SubType = UInt32(source, "subType"),
        NumVertices = UInt32(source, "numVertices"),
        NumTriangles = UInt32(source, "numTriangles")
    };

    static HclTransformSetDefinition ReadTransformSet(HkObject source) => new()
    {
        Name = String(source, "name"),
        Type = Int32(source, "type"),
        NumTransforms = UInt32(source, "numTransforms")
    };

    static HkaAnimationContainer ReadAnimations(HkObject source)
    {
        var result = new HkaAnimationContainer();
        foreach (HkObject obj in Items(source, "skeletons").OfType<HkObject>())
        {
            var skeleton = new HkaSkeleton { Name = String(obj, "name") };
            short[] parents = Array<short>(obj, "parentIndices");
            object?[] bones = Items(obj, "bones").ToArray();
            for (int i = 0; i < Math.Max(parents.Length, bones.Length); i++)
                skeleton.Bones.Add(new HkaBone
                {
                    Name = i < bones.Length ? String(bones[i], "name") : string.Empty,
                    ParentIndex = i < parents.Length ? parents[i] : (short)-1
                });
            float[] pose = Floats(Field(obj, "referencePose"));
            for (int i = 0; i + 11 < pose.Length; i += 12)
            {
                var translation = new Vector3(pose[i], pose[i + 1], pose[i + 2]);
                var rotation = new Quaternion(pose[i + 4], pose[i + 5], pose[i + 6], pose[i + 7]);
                var scale = new Vector3(pose[i + 8], pose[i + 9], pose[i + 10]);
                skeleton.ReferencePose.Add(Matrix4x4.CreateScale(scale) *
                    Matrix4x4.CreateFromQuaternion(rotation) * Matrix4x4.CreateTranslation(translation));
            }
            result.Skeletons.Add(skeleton);
        }
        return result;
    }

    static object? Field(object? source, string name)
    {
        HkStruct? fields = source switch { HkObject o => o.Fields, HkStruct s => s, _ => null };
        if (fields is null || !fields.Members.Any(m => m.Name == name)) return null;
        return fields[name];
    }

    static IEnumerable<object?> Items(object? source, string name) => Items(Field(source, name));
    static IEnumerable<object?> Items(object? value) => value is Array a ? a.Cast<object?>() : [];

    static T[] Array<T>(object? source, string name) => Array<T>(Field(source, name));
    static T[] Array<T>(object? value)
    {
        if (value is T[] exact) return exact;
        if (value is not Array a) return [];
        var result = new T[a.Length];
        for (int i = 0; i < result.Length; i++) result[i] = (T)Convert.ChangeType(a.GetValue(i)!, typeof(T));
        return result;
    }

    static float[] Floats(object? value)
    {
        if (value is float[] exact) return exact;
        if (value is not Array a) return [];
        var result = new float[a.Length];
        for (int i = 0; i < result.Length; i++) result[i] = Convert.ToSingle(a.GetValue(i));
        return result;
    }

    static Vector3 Vector3(object? value)
    {
        // An hkPackedVector3 arrives from HkxSharp as a struct with one member, "values": four s16s. Without this the
        // packed local positions of an hclObjectSpaceSkinPOperator came back as zeros, which put every anchor of a
        // cloth piece on its bone's origin.
        if (value is HkStruct wrapped && wrapped.Members.Any(m => m.Name == "values"))
            value = wrapped["values"];

        if (value is short[] packed && packed.Length >= 4)
        {
            var (x, y, z) = HkPackedVector3.Unpack(packed);
            return new Vector3(x, y, z);
        }

        float[] f = Floats(value);
        if (f.Length >= 3) return new Vector3(f[0], f[1], f[2]);
        return System.Numerics.Vector3.Zero;
    }

    static Vector4 Vector4(object? value)
    {
        float[] f = Floats(value);
        return f.Length >= 4 ? new Vector4(f[0], f[1], f[2], f[3]) : System.Numerics.Vector4.Zero;
    }

    /// <summary>The w of each float4 of a local-position array: a bone-space deformer's blend weights.</summary>
    static float[] LocalW(object? value)
    {
        if (value is not float[] f || f.Length % 4 != 0) return [];
        var result = new float[f.Length / 4];
        for (int i = 0; i < result.Length; i++) result[i] = f[i * 4 + 3];
        return result;
    }

    static Vector3[] Vector3Array(object? value)
    {
        if (value is float[] f)
        {
            int stride = f.Length % 4 == 0 ? 4 : 3;
            var result = new Vector3[f.Length / stride];
            for (int i = 0; i < result.Length; i++) result[i] = new Vector3(f[i * stride], f[i * stride + 1], f[i * stride + 2]);
            return result;
        }
        return Items(value).Select(Vector3).ToArray();
    }

    static Matrix4x4 Matrix(object? value)
    {
        float[] f = Floats(value);
        if (f.Length < 16) return Matrix4x4.Identity;
        return new Matrix4x4(f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7],
            f[8], f[9], f[10], f[11], f[12], f[13], f[14], f[15]);
    }

    static Matrix4x4[] MatrixArray(object? value)
    {
        float[] f = Floats(value);
        var result = new Matrix4x4[f.Length / 16];
        for (int i = 0; i < result.Length; i++) result[i] = Matrix(f.AsSpan(i * 16, 16).ToArray());
        return result;
    }

    static string String(object? source, string name) => Field(source, name) as string ?? string.Empty;
    static bool Bool(object? source, string name, bool fallback = false) => ConvertValue(Field(source, name), fallback);
    static byte Byte(object? source, string name, byte fallback = 0) => ConvertValue(Field(source, name), fallback);
    static sbyte SByte(object? source, string name, sbyte fallback = 0) => ConvertValue(Field(source, name), fallback);
    static ushort UInt16(object? source, string name, ushort fallback = 0) => ConvertValue(Field(source, name), fallback);
    static uint UInt32(object? source, string name, uint fallback = 0) => ConvertValue(Field(source, name), fallback);
    static int Int32(object? source, string name, int fallback = 0) => ConvertValue(Field(source, name), fallback);
    static float Float(object? source, string name, float fallback = 0) => ConvertValue(Field(source, name), fallback);
    static uint UInt32(object? value, uint fallback = 0) => ConvertValue(value, fallback);

    static T ConvertValue<T>(object? value, T fallback)
    {
        if (value is T exact) return exact;
        if (value is HkStruct s)
        {
            object? first = s.Values.FirstOrDefault(v => v is not null);
            return ConvertValue(first, fallback);
        }
        try { return value is null ? fallback : (T)Convert.ChangeType(value, typeof(T)); }
        catch (Exception) { return fallback; }
    }
}
