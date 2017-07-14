from yattag import Doc, indent

doc, tag, text = Doc().tagtext()

with tag('robot'):
    with tag('doc'):
        with tag('field1', name='blah'):
            text('some value1')
        with tag('xacro:macro', name='asdfasd'):
            text('some value2')

result = indent(
    doc.getvalue(),
    indentation = ' '*4,
    newline = '\r\n'
)
print(result)


with open('output.xml','a') as f:
    f.write(result)

