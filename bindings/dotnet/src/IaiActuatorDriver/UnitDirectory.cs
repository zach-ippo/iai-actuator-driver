using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;

namespace IaiActuatorDriver
{
    /// <summary>Index table header at 0x0000 (MODL, DQNT, LQNT, BFRD, BFTD).</summary>
    public sealed record IndexHeader(int ModelCode, int WordsPerRecord, int RecordCount, int RxBuffer, int TxBuffer)
    {
        /// <summary>Decode the first five header words.</summary>
        public static IndexHeader FromWords(IReadOnlyList<ushort> words)
        {
            if (words is null || words.Count < 5)
            {
                throw new ArgumentException("need 5 header words", nameof(words));
            }

            return new IndexHeader(words[0], words[1], words[2], words[3], words[4]);
        }

        /// <summary>The layout this driver understands: 8 words per record, 1..64 records.</summary>
        public bool LayoutOk => WordsPerRecord == 8 && RecordCount > 0 && RecordCount <= 64;
    }

    /// <summary>One 8-word category record of the index table (CI, MD, DQ, LQ, DA, IA, BA, MS).</summary>
    /// <param name="CategoryId">CI: the class id.</param>
    /// <param name="ModelCode">MD: flags.</param>
    /// <param name="MemberCount">DQ: members per entry.</param>
    /// <param name="ListCount">LQ: entries.</param>
    /// <param name="DataAddress">DA: first register.</param>
    /// <param name="IdListAddress">IA: the ID map ((item id, 0) pairs), 0 = none.</param>
    /// <param name="CoilBase">BA: coil base, 0 = none.</param>
    /// <param name="MemberSize">MS: registers per member, 0 = mixed sizes.</param>
    public sealed record IndexCategory(int CategoryId, int ModelCode, int MemberCount, int ListCount, int DataAddress, int IdListAddress, int CoilBase, int MemberSize)
    {
        /// <summary>Register extent of the block (<see cref="Registers.CategoryExtent"/>).</summary>
        public BlockExtent Extent => Registers.CategoryExtent(CategoryId, MemberCount, ListCount, MemberSize);

        /// <summary>What the class holds ("?" when not understood).</summary>
        public string Name => Registers.CATEGORY_NAMES.TryGetValue(CategoryId, out var name) ? name : "?";

        /// <summary>Decode one 8-word record.</summary>
        public static IndexCategory FromWords(IReadOnlyList<ushort> words, int offset = 0)
        {
            if (words is null || offset < 0 || offset + 8 > words.Count)
            {
                throw new ArgumentException("need 8 record words", nameof(words));
            }

            return new IndexCategory(
                words[offset], words[offset + 1], words[offset + 2], words[offset + 3],
                words[offset + 4], words[offset + 5], words[offset + 6], words[offset + 7]);
        }
    }

    /// <summary>One member of a directory class (16-bit for MS = 1, 32-bit high word first for MS = 2).</summary>
    public sealed record ClassItem(int ClassId, int? ItemId, int Entry, int Index, int Address, int Size, uint Raw)
    {
        /// <summary>The value as a signed 16- or 32-bit number.</summary>
        public int Signed => Size == 1 ? unchecked((short)Raw) : unchecked((int)Raw);

        /// <summary>The value in hex, 4 digits per register.</summary>
        public string Hex => Raw.ToString("X" + (4 * Size).ToString(CultureInfo.InvariantCulture), CultureInfo.InvariantCulture);

        /// <summary>"&lt;class&gt;.&lt;item&gt;" (class in hex above 9), plus "[entry]" for multi-entry classes.</summary>
        public string Key
        {
            get
            {
                var cls = ClassId > 9 ? ClassId.ToString("X", CultureInfo.InvariantCulture) : ClassId.ToString(CultureInfo.InvariantCulture);
                var item = ItemId.HasValue ? ItemId.Value.ToString(CultureInfo.InvariantCulture) : "?";
                return $"{cls}.{item}" + (Entry != 0 ? $"[{Entry}]" : string.Empty);
            }
        }

        /// <summary>Signed value x 0.01: the unit's usual mm / mm/s / G scale (a convenience).</summary>
        public double Scaled => Math.Round(Signed * 0.01, 2);
    }

    /// <summary>A directory class read by <see cref="UnitDirectory.ReadClass"/>.</summary>
    public sealed record ClassBlock(int ClassId, int Address, int Words, int MemberSize, int Entries, int? IdMapAddress, IReadOnlyList<ClassItem> Items)
    {
        /// <summary>The item with this ID (<c>null</c> if this unit's class has no such item).</summary>
        public ClassItem? Item(int itemId, int entry = 0) => Items.FirstOrDefault(i => i.ItemId == itemId && i.Entry == entry);
    }

    /// <summary>
    /// The unit's directory (index table at 0x0000): a header and one record per register class. The driver finds every
    /// block through it, so the same parameter is found by class + item ID whatever its address on this model (class 8
    /// item 50 is 0x083A on the gripper, 0x083E on the slider; VERIFIED). Only FC03 is ever sent, and every read stays
    /// inside a block the unit itself declares: the unit does not reject unmapped addresses (it returns 0000), so these
    /// bounds are the only guard against reading past a block.
    /// </summary>
    public sealed class UnitDirectory
    {
        /// <summary>Largest class <see cref="ReadClass"/> reads in one go (the position table, 12288 words, is read row by row).</summary>
        public const int MaxClassWords = 1024;

        /// <summary>Wrap a header and records already read.</summary>
        public UnitDirectory(IndexHeader? header, IEnumerable<IndexCategory> records)
        {
            Header = header;
            Records = (records ?? throw new ArgumentNullException(nameof(records))).ToList().AsReadOnly();
        }

        /// <summary>The header (<c>null</c> when the records came from elsewhere).</summary>
        public IndexHeader? Header { get; }

        /// <summary>The category records, in the unit's order.</summary>
        public IReadOnlyList<IndexCategory> Records { get; }

        /// <summary>
        /// Read the index table (header, then LQNT records of DQNT words; FC03 only). Throws
        /// <see cref="InvalidDataException"/> when the header has an unexpected layout.
        /// </summary>
        public static UnitDirectory Read(IRegisterReader reader)
        {
            if (reader is null)
            {
                throw new ArgumentNullException(nameof(reader));
            }

            var header = IndexHeader.FromWords(reader.ReadRegisters(Registers.INDEX_HEADER, Registers.INDEX_HEADER_WORDS));
            if (!header.LayoutOk)
            {
                throw new InvalidDataException($"index table: unexpected layout {header}");
            }

            var words = reader.ReadRegisters(Registers.INDEX_RECORDS, 8 * header.RecordCount);
            return new UnitDirectory(header, ParseRecords(words));
        }

        /// <summary>Decode a whole index table dump: 8 header words, then the records.</summary>
        public static UnitDirectory FromWords(IReadOnlyList<ushort> words)
        {
            var header = IndexHeader.FromWords(words);
            if (!header.LayoutOk)
            {
                throw new InvalidDataException($"index table: unexpected layout {header}");
            }

            var need = Registers.INDEX_HEADER_WORDS + (8 * header.RecordCount);
            if (words.Count < need)
            {
                throw new InvalidDataException($"index table: {words.Count} words, the header announces {need}");
            }

            return new UnitDirectory(header, ParseRecords(words.Skip(Registers.INDEX_HEADER_WORDS).Take(8 * header.RecordCount).ToArray()));
        }

        /// <summary>Decode consecutive 8-word records (a trailing partial record is ignored).</summary>
        public static IReadOnlyList<IndexCategory> ParseRecords(IReadOnlyList<ushort> words)
        {
            var records = new List<IndexCategory>();
            for (var i = 0; i + 8 <= words.Count; i += 8)
            {
                records.Add(IndexCategory.FromWords(words, i));
            }

            return records.AsReadOnly();
        }

        /// <summary>The record of <paramref name="classId"/>, or <c>null</c>.</summary>
        public IndexCategory? Find(int classId) => Records.FirstOrDefault(r => r.CategoryId == classId);

        /// <summary>The record of <paramref name="classId"/>; throws <see cref="KeyNotFoundException"/> when the unit has no such class.</summary>
        public IndexCategory Get(int classId) =>
            Find(classId) ?? throw new KeyNotFoundException($"class 0x{classId:X4} is not in this unit's directory");

        /// <summary>
        /// How many words of a block a read-only browser may read: its extent, or for a block of uncertain size the largest
        /// extent its record allows, never into the next directory block. Words past the certain extent are "size uncertain".
        /// </summary>
        public int ReadableWords(IndexCategory record) => ReadableWords(record, Records);

        /// <inheritdoc cref="ReadableWords(IndexCategory)"/>
        public static int ReadableWords(IndexCategory record, IEnumerable<IndexCategory>? records)
        {
            var extent = record.Extent;
            if (extent.Exact)
            {
                return extent.Words;
            }

            var wordsMax = extent.WordsMax;
            var following = (records ?? Enumerable.Empty<IndexCategory>()).Where(r => r.DataAddress > record.DataAddress).Select(r => r.DataAddress).ToList();
            if (following.Count > 0)
            {
                wordsMax = Math.Min(wordsMax, following.Min() - record.DataAddress);
            }

            return Math.Max(extent.Words, wordsMax);
        }

        /// <summary>
        /// Raw words of block <paramref name="classId"/> (FC03), <paramref name="count"/> words from <paramref name="offset"/>
        /// (<c>null</c>: to the end of the certain extent). The read must stay inside the block (<see cref="ReadableWords(IndexCategory)"/>);
        /// an out-of-range request throws <see cref="ArgumentOutOfRangeException"/> and nothing is read.
        /// </summary>
        public ushort[] ReadBlock(IRegisterReader reader, int classId, int offset = 0, int? count = null)
        {
            var rec = Get(classId);
            var words = rec.Extent.Words;
            var readable = ReadableWords(rec);
            var n = count ?? (words - offset);
            if (offset < 0 || n < 1 || offset + n > readable)
            {
                var size = readable == words ? $"{words} registers" : $"{words}-{readable} registers (size uncertain)";
                throw new ArgumentOutOfRangeException(
                    nameof(count), $"class 0x{classId:X4} has {size} at 0x{rec.DataAddress:X4}: offset {offset} + count {n} is outside it");
            }

            return reader.ReadRegisters(rec.DataAddress + offset, n);
        }

        /// <summary>
        /// Read class <paramref name="classId"/> and label every member with its item ID (FC03 only). The data (DQ x MS x
        /// LQ registers at DA) and the ID map ((item id, 0) pairs at IA) come from the unit's own directory. A class
        /// without an ID map, or with mixed member sizes (MS = 0), is returned word by word with no item ID. Throws
        /// <see cref="KeyNotFoundException"/> if the class is not in the directory and
        /// <see cref="ArgumentOutOfRangeException"/> if it is larger than <paramref name="maxWords"/>.
        /// </summary>
        public ClassBlock ReadClass(IRegisterReader reader, int classId, int maxWords = MaxClassWords)
        {
            var rec = Get(classId);
            var words = rec.Extent.Words;
            if (words > maxWords)
            {
                throw new ArgumentOutOfRangeException(nameof(maxWords), $"class 0x{classId:X4} has {words} registers (> {maxWords}); read it in parts");
            }

            var data = words > 0 ? reader.ReadRegisters(rec.DataAddress, words) : Array.Empty<ushort>();
            var uniform = rec.MemberSize == 1 || rec.MemberSize == 2;
            var size = uniform ? rec.MemberSize : 1;
            var perEntry = uniform ? rec.MemberCount * size : words;
            ushort[]? ids = null;
            if (rec.IdListAddress != 0 && uniform && rec.MemberCount > 0)
            {
                var idWords = reader.ReadRegisters(rec.IdListAddress, 2 * rec.MemberCount);
                ids = Enumerable.Range(0, rec.MemberCount).Select(i => idWords[2 * i]).ToArray();
            }

            var items = new List<ClassItem>();
            for (var n = 0; n + size <= data.Length; n += size)
            {
                var entry = perEntry > 0 ? n / perEntry : 0;
                var index = (perEntry > 0 ? n % perEntry : n) / size;
                var raw = size == 1 ? data[n] : ((uint)data[n] << 16) | data[n + 1];
                int? itemId = ids != null && index < ids.Length ? ids[index] : (int?)null;
                items.Add(new ClassItem(classId, itemId, entry, index, rec.DataAddress + n, size, raw));
            }

            return new ClassBlock(classId, rec.DataAddress, words, rec.MemberSize, rec.ListCount, rec.IdListAddress != 0 ? rec.IdListAddress : (int?)null, items.AsReadOnly());
        }

        /// <summary>
        /// Class 1 support-function words by name (SUPT, DIAG, MVPT, SPTS, CTYP, SUP2, ... SUP3; FC03 only). Class 1 has no
        /// ID map and a fixed member order (<see cref="Registers.SUPPORT_MEMBERS"/>, names UNVERIFIED); the directory gives
        /// the member count (EC #3: 6, up to SUP2; slider / gripper 11, up to SUP3). <c>null</c> when the class is not in
        /// the directory. An exception response propagates (<see cref="ModbusSlaveException"/>).
        /// </summary>
        public IReadOnlyDictionary<string, uint>? ReadSupportWords(IRegisterReader reader)
        {
            var rec = Find(Registers.CATEGORY_SUPPORT);
            if (rec is null || rec.MemberCount < 1)
            {
                return null;
            }

            var members = Registers.SUPPORT_MEMBERS.Take(rec.MemberCount).ToList();
            var words = reader.ReadRegisters(rec.DataAddress, members.Sum(m => m.Words));
            var result = new Dictionary<string, uint>();
            var offset = 0;
            foreach (var (name, size) in members)
            {
                result[name] = size == 1 ? words[offset] : ((uint)words[offset] << 16) | words[offset + 1];
                offset += size;
            }

            return result;
        }

        /// <summary>ACCL (0x0612, u32; bits 8-11 = extensibility level) from the identification block; <c>null</c> without class 6. FC03 only.</summary>
        public uint? ReadAccl(IRegisterReader reader)
        {
            var rec = Find(Registers.CATEGORY_IDENTIFICATION);
            if (rec is null)
            {
                return null;
            }

            var words = reader.ReadRegisters(rec.DataAddress + Registers.IDENT_ACCL, 2);
            return Registers.FromU32Words(words[0], words[1]);
        }
    }
}
